from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import cast

from fastai.learner import load_learner

from agent import UnrealAgent
from data_collection.capture import capture_lit_image, resolve_lit_camera_id
from data_collection.pawn_control import set_pawn_rotation, teleport_pawn
from data_collection.spline import CurveMode, load_path_from_map


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _policy_anchor_xy(
    x: float,
    y: float,
    yaw_deg: float,
    *,
    forward_cm: float,
    right_cm: float,
) -> tuple[float, float]:
    if forward_cm == 0.0 and right_cm == 0.0:
        return x, y
    rad = math.radians(yaw_deg)
    fx = math.cos(rad)
    fy = math.sin(rad)
    rx = -math.sin(rad)
    ry = math.cos(rad)
    return x + forward_cm * fx + right_cm * rx, y + forward_cm * fy + right_cm * ry


def _set_camera_pose(
    agent: UnrealAgent,
    camera_id: int,
    *,
    x: float,
    y: float,
    z: float,
    pitch_deg: float,
    yaw_deg: float,
    roll_deg: float,
    debug: bool,
) -> None:
    try:
        cmd_loc = f"vset /camera/{camera_id}/location {x:.4f} {y:.4f} {z:.4f}"
        cmd_rot = (
            f"vset /camera/{camera_id}/rotation "
            f"{pitch_deg:.4f} {yaw_deg:.4f} {roll_deg:.4f}"
        )
        if debug:
            print(f"  >> {cmd_loc}")
        agent.client.request(cmd_loc)
        if debug:
            print(f"  >> {cmd_rot}")
        agent.client.request(cmd_rot)
    except Exception as e:
        if debug:
            print(f"  !! camera follow failed: {e}")


def _nearest_centerline_dist_xy(spline, x: float, y: float, samples: int) -> tuple[float, float]:
    """Approximate nearest centerline distance in XY; returns (distance_cm, nearest_s)."""
    total = spline.total_length
    if total <= 0:
        return 0.0, 0.0
    best_d = float("inf")
    best_s = 0.0
    n = max(16, int(samples))
    for i in range(n + 1):
        s = (i / n) * total
        p = spline.sample(s)
        d = math.hypot(x - p.x, y - p.y)
        if d < best_d:
            best_d = d
            best_s = s
    return best_d, best_s


def _centerline_state(spline, x: float, y: float, samples: int) -> tuple[float, float, float]:
    """Return (nearest_s, signed_cross_track_cm, tangent_yaw_deg)."""
    total = spline.total_length
    if total <= 0:
        return 0.0, 0.0, 0.0
    nearest_s = 0.0
    best_d = float("inf")
    n = max(16, int(samples))
    for i in range(n + 1):
        s = (i / n) * total
        p = spline.sample(s)
        d = math.hypot(x - p.x, y - p.y)
        if d < best_d:
            best_d = d
            nearest_s = s

    p0 = spline.sample(nearest_s)
    ds = max(1e-3, min(5.0, total * 0.002))
    p1 = spline.sample(min(total, nearest_s + ds))
    tangent_x = p1.x - p0.x
    tangent_y = p1.y - p0.y
    tangent_norm = math.hypot(tangent_x, tangent_y)
    if tangent_norm < 1e-6:
        tangent_yaw = p0.yaw_deg
        signed_cross = 0.0
        return nearest_s, signed_cross, tangent_yaw

    tangent_x /= tangent_norm
    tangent_y /= tangent_norm
    off_x = x - p0.x
    off_y = y - p0.y
    # 2D cross(tangent, offset): + means agent is left of path tangent.
    signed_cross = tangent_x * off_y - tangent_y * off_x
    tangent_yaw = math.degrees(math.atan2(tangent_y, tangent_x))
    return nearest_s, signed_cross, tangent_yaw


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Evaluate a trained left/right/forward model in closed loop: "
            "capture image -> predict action -> move -> repeat."
        )
    )
    p.add_argument("--model", default="models/turn_classifier.pkl", help="Path to exported fastai model.")
    p.add_argument("--map", default="data/maps/map_w.json", help="Map JSON with path in world space.")
    p.add_argument("--curve", choices=("catmull_rom", "polyline"), default="catmull_rom")
    p.add_argument("--curve-samples", type=int, default=48)
    p.add_argument("--max-steps", type=int, default=300, help="Maximum policy steps.")
    p.add_argument("--forward-step-cm", type=float, default=12.5, help="Forward motion per 'forward' action.")
    p.add_argument("--turn-deg", type=float, default=5.0, help="Yaw change for left/right actions.")
    p.add_argument(
        "--centering-assist",
        action="store_true",
        help="Bias control toward re-centering on the tape using centerline geometry.",
    )
    p.add_argument(
        "--centering-cross-track-cm",
        type=float,
        default=3.0,
        help="Trigger centering turn when |cross_track| exceeds this threshold.",
    )
    p.add_argument(
        "--centering-heading-deg",
        type=float,
        default=3.0,
        help="Trigger centering turn when |heading error| exceeds this threshold.",
    )
    p.add_argument(
        "--centering-priority",
        choices=("always", "model_must_agree", "off"),
        default="always",
        help=(
            "always: geometry override has priority; "
            "model_must_agree: only force turn if model predicts same side; "
            "off: disable centering override."
        ),
    )
    p.add_argument(
        "--finish-mode",
        choices=("distance", "progress", "lap"),
        default="distance",
        help=(
            "distance: finish when XY distance to spline end <= --finish-threshold-cm; "
            "progress: finish when progress ratio >= --target-progress-ratio and min-progress gate is met; "
            "lap: finish after unwrapped progress covers --target-laps * total_length."
        ),
    )
    p.add_argument("--finish-threshold-cm", type=float, default=30.0, help="Success if end distance <= threshold.")
    p.add_argument(
        "--target-progress-ratio",
        type=float,
        default=0.98,
        help="Target progress ratio for finish-mode=progress (default: 0.98).",
    )
    p.add_argument(
        "--min-progress-ratio",
        type=float,
        default=0.80,
        help=(
            "Gate for distance/progress finish checks: ignore endpoint closeness "
            "until progress reaches this ratio. Helps looped tracks."
        ),
    )
    p.add_argument(
        "--target-laps",
        type=float,
        default=1.0,
        help="Number of laps for finish-mode=lap (default: 1.0).",
    )
    p.add_argument("--centerline-samples", type=int, default=2000, help="Samples for nearest centerline distance.")
    p.add_argument("--output-dir", default="data/evals", help="Folder for eval logs/images.")
    p.add_argument("--run-name", default="eval_run")
    p.add_argument("--camera-id", type=int, default=None)
    p.add_argument("--spawn-capture-camera", action="store_true")
    p.add_argument("--settle", type=float, default=0.08, help="Sleep after action before capture.")
    p.add_argument("--force-camera-follow", action="store_true")
    p.add_argument("--camera-x-offset-cm", type=float, default=0.0)
    p.add_argument("--camera-y-offset-cm", type=float, default=0.0)
    p.add_argument("--camera-z-offset-cm", type=float, default=0.0)
    p.add_argument("--camera-pitch-deg", type=float, default=0.0)
    p.add_argument("--camera-roll-deg", type=float, default=0.0)
    p.add_argument("--pawn", default=None)
    p.add_argument("--pawn-z", type=float, default=None)
    p.add_argument(
        "--policy-anchor-forward-cm",
        type=float,
        default=0.0,
        help="Centerline metrics use pawn XY plus this offset along pawn forward (same as collect.py).",
    )
    p.add_argument(
        "--policy-anchor-right-cm",
        type=float,
        default=0.0,
        help="Offset along pawn right for centerline metrics (+ = actor's right).",
    )
    p.add_argument("--debug", action="store_true")
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    out_dir = Path(args.output_dir) / args.run_name
    images_dir = out_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / "eval_log.jsonl"
    summary_path = out_dir / "summary.json"

    try:
        learn = load_learner(Path(args.model))
    except Exception as e:
        print(f"Failed to load model {args.model}: {e}", file=sys.stderr)
        return 2

    spline = load_path_from_map(
        args.map,
        curve=cast(CurveMode, args.curve),
        samples_per_segment=args.curve_samples,
    )
    end = spline.sample(spline.total_length)
    total_len = float(spline.total_length)

    agent = UnrealAgent()
    if not agent.connect():
        print("Failed to connect to UnrealCV.", file=sys.stderr)
        return 2

    try:
        cam_id = args.camera_id
        if args.spawn_capture_camera:
            try:
                resp = agent.client.request("vset /cameras/spawn")
                if args.debug:
                    print(f"spawn camera response: {resp!r}")
            except Exception as e:
                if args.debug:
                    print(f"spawn camera failed: {e}")
        if cam_id is None:
            cam_id = resolve_lit_camera_id(agent, debug=args.debug)
        if cam_id is None:
            print("Could not resolve camera id.", file=sys.stderr)
            return 2

        pawn_z = args.pawn_z
        if pawn_z is None:
            pose0 = agent.get_pawn_pose()
            pawn_z = pose0.z if pose0 is not None else 0.0

        # Bootstrap to path start (same convention as collect.py).
        start = spline.sample(0.0)
        start_yaw = _wrap_deg(start.yaw_deg)
        teleport_pawn(
            agent,
            start.x,
            start.y,
            start_yaw,
            z=pawn_z,
            pawn=args.pawn,
            debug=args.debug,
        )
        if args.settle > 0:
            time.sleep(args.settle)

        counts = {"forward": 0, "left": 0, "right": 0, "other": 0}
        last_dist_to_end = float("inf")
        max_center = 0.0
        sum_center = 0.0
        steps = 0
        last_nearest_s: float | None = None
        s_unwrapped = 0.0
        finish_reason: str | None = None

        with open(log_path, "w", encoding="utf-8") as log_f:
            for step_idx in range(args.max_steps):
                pose = agent.get_pawn_pose()
                if pose is None:
                    print("Could not read pawn pose.", file=sys.stderr)
                    return 2

                if args.force_camera_follow:
                    yaw_rad = math.radians(pose.yaw)
                    world_dx = args.camera_x_offset_cm * math.cos(yaw_rad) - args.camera_y_offset_cm * math.sin(yaw_rad)
                    world_dy = args.camera_x_offset_cm * math.sin(yaw_rad) + args.camera_y_offset_cm * math.cos(yaw_rad)
                    _set_camera_pose(
                        agent,
                        cam_id,
                        x=pose.x + world_dx,
                        y=pose.y + world_dy,
                        z=pawn_z + args.camera_z_offset_cm,
                        pitch_deg=args.camera_pitch_deg,
                        yaw_deg=pose.yaw,
                        roll_deg=args.camera_roll_deg,
                        debug=args.debug,
                    )

                img_path = images_dir / f"step_{step_idx:05d}.png"
                if not capture_lit_image(agent, img_path, camera_id=cam_id, debug=args.debug):
                    print(f"Failed to capture {img_path}", file=sys.stderr)
                    return 2

                pred, _, _ = learn.predict(img_path)
                model_action = str(pred).lower()
                action = model_action

                ax, ay = _policy_anchor_xy(
                    pose.x,
                    pose.y,
                    pose.yaw,
                    forward_cm=args.policy_anchor_forward_cm,
                    right_cm=args.policy_anchor_right_cm,
                )

                # Measure before action for evaluation.
                center_dist, nearest_s = _nearest_centerline_dist_xy(
                    spline, ax, ay, args.centerline_samples
                )
                nearest_s2, signed_cross_track, tangent_yaw = _centerline_state(
                    spline, ax, ay, args.centerline_samples
                )
                heading_err = _wrap_deg(tangent_yaw - pose.yaw)
                if args.centering_priority != "off" and args.centering_assist:
                    need_center_turn = (
                        abs(signed_cross_track) > args.centering_cross_track_cm
                        or abs(heading_err) > args.centering_heading_deg
                    )
                    if need_center_turn:
                        desired_turn = "left" if heading_err < 0 else "right"
                        if args.centering_priority == "always":
                            action = desired_turn
                        elif args.centering_priority == "model_must_agree":
                            if model_action == desired_turn:
                                action = desired_turn
                if last_nearest_s is None:
                    s_unwrapped = nearest_s2
                else:
                    ds = nearest_s2 - last_nearest_s
                    # Unwrap loop crossings so progress stays monotonic.
                    if ds < -0.5 * total_len:
                        ds += total_len
                    elif ds > 0.5 * total_len:
                        ds -= total_len
                    s_unwrapped += max(0.0, ds)
                last_nearest_s = nearest_s2
                progress_ratio = (s_unwrapped / total_len) if total_len > 0 else 0.0
                dist_to_end = math.hypot(pose.x - end.x, pose.y - end.y)
                last_dist_to_end = dist_to_end
                max_center = max(max_center, center_dist)
                sum_center += center_dist
                steps += 1

                log_row = {
                    "step_idx": step_idx,
                    "image": str(img_path),
                    "prediction": model_action,
                    "chosen_action": action,
                    "pose_before": {"x": pose.x, "y": pose.y, "z": pose.z, "yaw_deg": pose.yaw},
                    "policy_anchor_xy": {"x": ax, "y": ay},
                    "centerline_distance_cm": center_dist,
                    "nearest_s": nearest_s2,
                    "signed_cross_track_cm": signed_cross_track,
                    "tangent_yaw_deg": tangent_yaw,
                    "heading_error_deg": heading_err,
                    "s_unwrapped": s_unwrapped,
                    "progress_ratio": progress_ratio,
                    "distance_to_end_cm": dist_to_end,
                }
                log_f.write(json.dumps(log_row) + "\n")
                log_f.flush()

                min_progress_ok = progress_ratio >= args.min_progress_ratio
                reached = False
                if args.finish_mode == "distance":
                    reached = min_progress_ok and (dist_to_end <= args.finish_threshold_cm)
                    if reached:
                        finish_reason = (
                            f"distance<=threshold with progress gate "
                            f"({dist_to_end:.2f} <= {args.finish_threshold_cm:.2f}, "
                            f"progress={progress_ratio:.3f})"
                        )
                elif args.finish_mode == "progress":
                    reached = min_progress_ok and (progress_ratio >= args.target_progress_ratio)
                    if reached:
                        finish_reason = (
                            f"progress reached target "
                            f"({progress_ratio:.3f} >= {args.target_progress_ratio:.3f})"
                        )
                elif args.finish_mode == "lap":
                    reached = s_unwrapped >= args.target_laps * total_len
                    if reached:
                        finish_reason = (
                            f"lap target reached "
                            f"(s_unwrapped={s_unwrapped:.2f} >= {args.target_laps * total_len:.2f})"
                        )

                if reached:
                    print(f"Finish condition met at step {step_idx}: {finish_reason}")
                    break

                if action == "forward":
                    yaw_rad = math.radians(pose.yaw)
                    nx = pose.x + args.forward_step_cm * math.cos(yaw_rad)
                    ny = pose.y + args.forward_step_cm * math.sin(yaw_rad)
                    teleport_pawn(
                        agent, nx, ny, pose.yaw, z=pawn_z, pawn=args.pawn, debug=args.debug
                    )
                    counts["forward"] += 1
                elif action == "left":
                    set_pawn_rotation(
                        agent,
                        _wrap_deg(pose.yaw - args.turn_deg),
                        pawn=args.pawn,
                        debug=args.debug,
                    )
                    counts["left"] += 1
                elif action == "right":
                    set_pawn_rotation(
                        agent,
                        _wrap_deg(pose.yaw + args.turn_deg),
                        pawn=args.pawn,
                        debug=args.debug,
                    )
                    counts["right"] += 1
                else:
                    # Unknown class: safest fallback is forward.
                    yaw_rad = math.radians(pose.yaw)
                    nx = pose.x + args.forward_step_cm * math.cos(yaw_rad)
                    ny = pose.y + args.forward_step_cm * math.sin(yaw_rad)
                    teleport_pawn(
                        agent, nx, ny, pose.yaw, z=pawn_z, pawn=args.pawn, debug=args.debug
                    )
                    counts["other"] += 1

                if args.settle > 0:
                    time.sleep(args.settle)

        if args.finish_mode == "distance":
            success = (last_dist_to_end <= args.finish_threshold_cm) and (
                (s_unwrapped / total_len) >= args.min_progress_ratio if total_len > 0 else False
            )
        elif args.finish_mode == "progress":
            success = (
                (s_unwrapped / total_len) >= max(args.target_progress_ratio, args.min_progress_ratio)
                if total_len > 0
                else False
            )
        else:  # lap
            success = s_unwrapped >= args.target_laps * total_len
        summary = {
            "model": str(Path(args.model).resolve()),
            "map": str(Path(args.map).resolve()),
            "run_name": args.run_name,
            "max_steps": args.max_steps,
            "steps_executed": steps,
            "forward_step_cm": args.forward_step_cm,
            "turn_deg": args.turn_deg,
            "finish_mode": args.finish_mode,
            "finish_threshold_cm": args.finish_threshold_cm,
            "target_progress_ratio": args.target_progress_ratio,
            "min_progress_ratio": args.min_progress_ratio,
            "target_laps": args.target_laps,
            "finished_close_to_end": success,
            "finish_reason": finish_reason,
            "final_distance_to_end_cm": last_dist_to_end,
            "final_nearest_s": last_nearest_s,
            "final_s_unwrapped": s_unwrapped,
            "final_progress_ratio": (s_unwrapped / total_len) if total_len > 0 else None,
            "avg_centerline_distance_cm": (sum_center / steps) if steps > 0 else None,
            "max_centerline_distance_cm": max_center if steps > 0 else None,
            "action_counts": counts,
            "eval_log": str(log_path.resolve()),
        }
        summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"Saved eval log: {log_path}")
        print(f"Saved summary : {summary_path}")
        print(
            f"Finished close to end: {success} "
            f"(final dist {last_dist_to_end:.2f} cm, "
            f"avg center dist {summary['avg_centerline_distance_cm']:.2f} cm)"
            if summary["avg_centerline_distance_cm"] is not None
            else f"Finished close to end: {success}"
        )
        return 0
    finally:
        agent.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

