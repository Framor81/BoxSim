# Walk the pawn along a spline and collect frames.
#
# Pipeline (one frame per step):
#   1. Sample spline at distance s along it (+ look-ahead for yaw).
#   2. Teleport the pawn to that (x, y, yaw) over UnrealCV.
#   3. (Optional) wait for the physics/camera to settle.
#   4. (Optional) capture a lit PNG from the pawn camera.
#   5. Append a manifest row with pose + spline s + label.
#   6. s += step_cm, repeat until end of spline.
#
# The path comes from a BoxSim map JSON (map.json / map_w.json). The path
# originates in the UE5 Path Tape prototype plugin; BoxSim's map builder
# exports it under the "path" key in world (UE cm) coordinates. By default we
# follow a Catmull-Rom curve through those knots (see spline.py); use
# --curve polyline for chord-only motion.

from __future__ import annotations

import argparse
import json
import math
import random
import re
import sys
import time
from pathlib import Path
from typing import cast

# Importing the sibling package directly when run as a script: add the
# BoxSim repo root to sys.path if needed.
_THIS = Path(__file__).resolve()
_REPO = _THIS.parent.parent
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from agent import UnrealAgent  # noqa: E402
from data_collection.capture import (  # noqa: E402
    capture_lit_image,
    resolve_lit_camera_id,
)
from data_collection.pawn_control import (  # noqa: E402
    set_pawn_location,
    set_pawn_rotation,
)
from data_collection.spline import (  # noqa: E402
    CurveMode,
    load_path_from_map,
)


DEFAULT_MAP = "data/maps/map_w.json"
DEFAULT_OUTPUT_ROOT = "data/datasets"
DEFAULT_STEP_CM = 25.0
DEFAULT_LOOKAHEAD_CM = 75.0
DEFAULT_SETTLE_S = 0.12
DEFAULT_TURN_THRESHOLD_DEG = 15.0
DEFAULT_ALIGN_THRESHOLD_DEG = 5.0
DEFAULT_CURVE_SAMPLES_PER_SEGMENT = 48
DEFAULT_WAYPOINT_RADIUS_CM = 0.0  # off by default; densely-sampled paths would
# otherwise label every frame 'waypoint'. Pass --waypoint-radius-cm <r> to enable.
DEFAULT_RANDOM_JITTER_XY_CM = 0.75
DEFAULT_RANDOM_JITTER_YAW_DEG = 1.0
DEFAULT_TURN_STEP_DEG = 2.5
DEFAULT_CENTER_TOL_CM = 6.0
DEFAULT_HEADING_TOL_DEG = 4.0
DEFAULT_MAX_CONSEC_TURNS = 2
DEFAULT_POST_TURN_FORWARD_SCALE = 0.5
# With --follow-spline-forward (no yaw snap), after max consecutive turns the policy
# used to force "forward" even when badly misaligned — long forward streaks on curves.
DEFAULT_TURN_BUDGET_ESCAPE_HEADING_MULT = 2.5


def _pick_turn_label(cross_track: float, heading_err: float) -> str:
    """Geometry-only turn: line left of anchor => turn left to approach."""
    if abs(cross_track) > 0.1:
        return "left" if cross_track > 0.0 else "right"
    return "left" if heading_err < 0.0 else "right"


def _parse_xyz(raw: str | None) -> tuple[float, float, float] | None:
    if not raw:
        return None
    nums = re.findall(r"-?\d+(?:\.\d+)?", str(raw))
    if len(nums) < 3:
        return None
    try:
        return (float(nums[0]), float(nums[1]), float(nums[2]))
    except ValueError:
        return None


def _list_camera_ids(agent: UnrealAgent) -> list[int]:
    try:
        raw = agent.client.request("vget /cameras")
    except Exception:
        raw = None
    ids: list[int] = []
    if raw:
        for tok in str(raw).replace(",", " ").split():
            try:
                ids.append(int(tok.strip()))
            except ValueError:
                continue
    if not ids:
        ids = [0, 1, 2, 3]
    return sorted(set(ids))


def _list_valid_camera_ids(agent: UnrealAgent) -> list[int]:
    """Drop ids that don't respond to vget /camera/<id>/location ('Invalid sensor id')."""
    valid: list[int] = []
    for cid in _list_camera_ids(agent):
        try:
            raw = agent.client.request(f"vget /camera/{cid}/location")
        except Exception:
            continue
        if raw and not str(raw).lower().startswith("error") and _parse_xyz(raw) is not None:
            valid.append(cid)
    return valid


def _read_camera_loc(
    agent: UnrealAgent, camera_id: int
) -> tuple[float, float, float] | None:
    try:
        raw = agent.client.request(f"vget /camera/{camera_id}/location")
    except Exception:
        raw = None
    return _parse_xyz(raw)


def _read_camera_rot(
    agent: UnrealAgent, camera_id: int
) -> tuple[float, float, float] | None:
    try:
        raw = agent.client.request(f"vget /camera/{camera_id}/rotation")
    except Exception:
        raw = None
    return _parse_xyz(raw)


def _is_camera_responsive(agent: UnrealAgent, cam_id: int) -> bool:
    try:
        raw = agent.client.request(f"vget /camera/{cam_id}/location")
    except Exception:
        return False
    if not raw or str(raw).lower().startswith("error"):
        return False
    return _parse_xyz(raw) is not None


def _spawn_capture_camera(
    agent: UnrealAgent, *, debug: bool = False
) -> int | None:
    """Ask UnrealCV to spawn a new camera. Return the new camera id or None.

    Only returns ids that actually respond to vget; some UE builds spawn a
    Fusion camera but never register it in /cameras, in which case we cannot
    safely target it and return None.
    """
    before = _list_valid_camera_ids(agent)
    if debug:
        print(f"  valid cameras before spawn: {before}")
    try:
        resp = agent.client.request("vset /cameras/spawn")
    except Exception as e:
        if debug:
            print(f"  !! vset /cameras/spawn failed: {e}")
        return None
    if debug:
        print(f"  << vset /cameras/spawn -> {resp!r}")

    after = _list_valid_camera_ids(agent)
    if debug:
        print(f"  valid cameras after spawn:  {after}")

    new_ids = [c for c in after if c not in before]
    candidates: list[int] = list(new_ids)
    if resp:
        for tok in str(resp).strip().split():
            try:
                candidates.append(int(tok))
            except ValueError:
                continue
    if before:
        candidates.append(max(before) + 1)

    seen: set[int] = set()
    for cid in candidates:
        if cid in seen:
            continue
        seen.add(cid)
        if _is_camera_responsive(agent, cid):
            if debug:
                print(f"  spawn picked responsive camera id = {cid}")
            return cid

    if debug:
        print("  spawn returned no responsive new id; cannot use spawned camera.")
    return None


def _probe_cameras(agent: UnrealAgent, output_dir: Path, *, debug: bool) -> int:
    probe_dir = output_dir / "camera_probe"
    probe_dir.mkdir(parents=True, exist_ok=True)
    print("\n=== Camera Probe Mode ===")
    print(f"Probe output dir: {probe_dir.resolve()}")

    objects = agent.list_objects()
    print(f"Objects ({len(objects)}):")
    for name in objects:
        print(f"  - {name}")

    pawn = agent.get_pawn_pose(debug=debug)
    if pawn is not None:
        print(
            f"Pawn pose: x={pawn.x:.2f} y={pawn.y:.2f} z={pawn.z:.2f} yaw={pawn.yaw:.2f}"
        )
    else:
        print("Pawn pose: unavailable")

    cam_ids = _list_camera_ids(agent)
    print(f"Cameras ({len(cam_ids)}): {cam_ids}")
    for cam_id in cam_ids:
        _print_camera_info(agent, cam_id, pawn=pawn)
        ok_write = _writability_test(agent, cam_id, debug=debug)
        print(f"    writable (vset honored): {ok_write}")

        out = probe_dir / f"camera_{cam_id}.png"
        ok = capture_lit_image(agent, out, camera_id=cam_id, debug=debug)
        if ok:
            print(f"    saved: {out.resolve()}")
        else:
            print(f"    capture failed for camera {cam_id}")

    print("\n--- Spawn test: vset /cameras/spawn ---")
    spawned = _spawn_capture_camera(agent, debug=debug)
    if spawned is None:
        print("  spawn failed (or this UnrealCV build does not support /cameras/spawn).")
    else:
        print(f"  spawned camera id = {spawned}")
        _print_camera_info(agent, spawned, pawn=pawn)
        ok_write = _writability_test(agent, spawned, debug=debug)
        print(f"    writable (vset honored): {ok_write}")
        out = probe_dir / f"camera_{spawned}_spawned.png"
        if capture_lit_image(agent, out, camera_id=spawned, debug=debug):
            print(f"    saved: {out.resolve()}")

    print("\nCamera probe complete. Compare camera_*.png to pick the correct camera.")
    print("Hints:")
    print("  - If only camera 0 exists and writable=False, use --spawn-capture-camera")
    print("    to dynamically spawn a free camera that follows the pawn.")
    print("  - Otherwise rerun with --camera-id <id> on a writable camera.")
    return 0


def _print_camera_info(
    agent: UnrealAgent, cam_id: int, *, pawn
) -> None:
    loc = _read_camera_loc(agent, cam_id)
    rot = _read_camera_rot(agent, cam_id)
    msg = f"  camera {cam_id}:"
    if loc is not None:
        msg += f" loc=({loc[0]:.2f}, {loc[1]:.2f}, {loc[2]:.2f})"
        if pawn is not None:
            dx = loc[0] - pawn.x
            dy = loc[1] - pawn.y
            msg += f" dist_xy={((dx * dx + dy * dy) ** 0.5):.2f}"
    else:
        msg += " loc=<unavailable>"
    if rot is not None:
        msg += f" rot(p,y,r)=({rot[0]:.2f}, {rot[1]:.2f}, {rot[2]:.2f})"
    else:
        msg += " rot=<unavailable>"
    print(msg)


def _writability_test(
    agent: UnrealAgent, camera_id: int, *, debug: bool
) -> bool:
    """Read camera pose, vset to a known offset, read back, restore. Returns True if vset took effect."""
    before = _read_camera_loc(agent, camera_id)
    if before is None:
        if debug:
            print("    writability: cannot read location; skipping.")
        return False
    target = (before[0] + 137.0, before[1] - 91.0, before[2] + 73.0)
    cmd = f"vset /camera/{camera_id}/location {target[0]:.4f} {target[1]:.4f} {target[2]:.4f}"
    try:
        if debug:
            print(f"    >> {cmd}")
        resp = agent.client.request(cmd)
        if debug:
            print(f"    << {resp!r}")
    except Exception as e:
        if debug:
            print(f"    !! write failed: {e}")
        return False
    time.sleep(0.05)
    after = _read_camera_loc(agent, camera_id)
    ok = False
    if after is not None:
        dx = abs(after[0] - target[0])
        dy = abs(after[1] - target[1])
        dz = abs(after[2] - target[2])
        ok = (dx + dy + dz) < 5.0
        if debug:
            print(
                f"    after-read loc=({after[0]:.2f}, {after[1]:.2f}, {after[2]:.2f}) "
                f"target=({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}) "
                f"|d|=({dx:.2f}, {dy:.2f}, {dz:.2f})"
            )
    # Restore original location (best effort).
    try:
        agent.client.request(
            f"vset /camera/{camera_id}/location {before[0]:.4f} {before[1]:.4f} {before[2]:.4f}"
        )
    except Exception:
        pass
    return ok


def _is_error_response(resp) -> bool:
    return resp is not None and str(resp).strip().lower().startswith("error")


def _set_camera_fov(
    agent: UnrealAgent, camera_id: int, fov_deg: float, *, debug: bool
) -> bool:
    cmd = f"vset /camera/{camera_id}/fov {fov_deg:.4f}"
    if debug:
        print(f"  >> {cmd}")
    try:
        resp = agent.client.request(cmd)
    except Exception as e:
        if debug:
            print(f"  !! fov set failed: {e}")
        return False
    if debug:
        print(f"  << {resp!r}")
    if _is_error_response(resp):
        print(f"  !! camera {camera_id} vset fov rejected: {str(resp).strip()}")
        return False
    return True


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
) -> bool:
    """Set camera pose. Returns False (and prints) if UnrealCV reports an error."""
    try:
        before = _read_camera_loc(agent, camera_id) if debug else None
        cmd_loc = f"vset /camera/{camera_id}/location {x:.4f} {y:.4f} {z:.4f}"
        cmd_rot = (
            f"vset /camera/{camera_id}/rotation "
            f"{pitch_deg:.4f} {yaw_deg:.4f} {roll_deg:.4f}"
        )
        if debug:
            print(f"  >> {cmd_loc}")
        r1 = agent.client.request(cmd_loc)
        if debug:
            print(f"  << {r1!r}")
        if _is_error_response(r1):
            print(
                f"  !! camera {camera_id} vset location rejected: {str(r1).strip()}"
            )
            return False
        if debug:
            print(f"  >> {cmd_rot}")
        r2 = agent.client.request(cmd_rot)
        if debug:
            print(f"  << {r2!r}")
        if _is_error_response(r2):
            print(
                f"  !! camera {camera_id} vset rotation rejected: {str(r2).strip()}"
            )
            return False
        if debug:
            after = _read_camera_loc(agent, camera_id)
            if after is not None:
                dx = abs(after[0] - x)
                dy = abs(after[1] - y)
                dz = abs(after[2] - z)
                honored = (dx + dy + dz) < 5.0
                print(
                    f"  cam{camera_id} loc before={before} target=({x:.2f},{y:.2f},{z:.2f}) "
                    f"after=({after[0]:.2f},{after[1]:.2f},{after[2]:.2f}) honored={honored}"
                )
        return True
    except Exception as e:
        if debug:
            print(f"  !! set camera pose failed: {e}")
        return False


def _label_for_turn(delta_yaw_deg: float, threshold_deg: float) -> str:
    if delta_yaw_deg > threshold_deg:
        return "right"
    if delta_yaw_deg < -threshold_deg:
        return "left"
    return "forward"


def _nearest_point_within(
    x: float,
    y: float,
    points: list[tuple[float, float]],
    radius_cm: float,
) -> tuple[float, float] | None:
    """Return (px, py) of the closest point within `radius_cm`, else None."""
    if not points or radius_cm <= 0.0:
        return None
    best: tuple[float, float] | None = None
    best_d = radius_cm
    for px, py in points:
        d = math.hypot(x - px, y - py)
        if d <= best_d:
            best_d = d
            best = (px, py)
    return best


def _nearest_centerline_state(
    spline, x: float, y: float, *, samples: int = 1200
) -> tuple[float, float, float, float]:
    """Approx nearest centerline point/tangent.

    Returns (nearest_s, nearest_x, nearest_y, nearest_yaw_deg).
    """
    total = float(spline.total_length)
    if total <= 0.0:
        p = spline.sample(0.0)
        return 0.0, p.x, p.y, p.yaw_deg
    n = max(64, int(samples))
    best_d = float("inf")
    best_s = 0.0
    for i in range(n + 1):
        s = (i / n) * total
        p = spline.sample(s)
        d = (x - p.x) ** 2 + (y - p.y) ** 2
        if d < best_d:
            best_d = d
            best_s = s
    p = spline.sample(best_s)
    return best_s, p.x, p.y, p.yaw_deg


def _wrap_deg(a: float) -> float:
    a = (a + 180.0) % 360.0 - 180.0
    return a


def _policy_anchor_xy(
    x: float,
    y: float,
    yaw_deg: float,
    *,
    forward_cm: float,
    right_cm: float,
) -> tuple[float, float]:
    """Shift (x,y) in pawn-local frame for centerline / cross-track only.

    Uses the same forward/right convention as camera offsets: forward = +X_body,
    right = +Y_body. World offset = forward*(cos,sin) + right*(-sin,cos).
    """
    if forward_cm == 0.0 and right_cm == 0.0:
        return x, y
    rad = math.radians(yaw_deg)
    fx = math.cos(rad)
    fy = math.sin(rad)
    rx = -math.sin(rad)
    ry = math.cos(rad)
    return x + forward_cm * fx + right_cm * rx, y + forward_cm * fy + right_cm * ry


def run(
    map_json: str,
    output_dir: Path,
    *,
    step_cm: float,
    lookahead_cm: float,
    settle_s: float,
    turn_threshold_deg: float,
    align_threshold_deg: float,
    turn_step_deg: float,
    center_tol_cm: float,
    heading_tol_deg: float,
    max_consecutive_turns: int,
    forward_step_cm: float | None,
    post_turn_forward_scale: float,
    pawn: str | None,
    pawn_z: float | None,
    camera_id: int | None,
    capture_enabled: bool,
    dry_run: bool,
    debug: bool,
    yaw_offset_deg: float,
    curve: CurveMode,
    samples_per_segment: int,
    jitter_xy_cm: float,
    jitter_yaw_deg: float,
    seed: int | None,
    probe_cameras: bool,
    force_camera_follow: bool,
    camera_x_offset_cm: float,
    camera_y_offset_cm: float,
    camera_z_offset_cm: float,
    camera_pitch_deg: float,
    camera_roll_deg: float,
    camera_fov_deg: float,
    spawn_capture_camera: bool,
    waypoint_radius_cm: float,
    skip_spline_bootstrap: bool,
    randomization: bool,
    policy_anchor_forward_cm: float,
    policy_anchor_right_cm: float,
    follow_spline_forward: bool,
    follow_spline_forward_sync_yaw: bool,
    turn_budget_escape_heading_mult: float,
    max_consecutive_forwards: int | None,
    finish_threshold_cm: float | None,
    min_finish_progress_ratio: float,
) -> int:
    spline = load_path_from_map(
        map_json,
        curve=curve,
        samples_per_segment=samples_per_segment,
    )
    waypoints: list[tuple[float, float]] = list(spline.points)
    if waypoint_radius_cm > 0.0:
        # Warn if the radius would swallow most steps (radius >= avg spacing/2).
        avg_spacing = (
            spline.total_length / max(1, len(waypoints) - 1)
            if len(waypoints) > 1
            else 0.0
        )
        msg = (
            f"Loaded {len(waypoints)} waypoint(s); snap/label radius = "
            f"{waypoint_radius_cm:.1f} cm (avg waypoint spacing ~ {avg_spacing:.1f} cm)"
        )
        if avg_spacing > 0.0 and waypoint_radius_cm >= avg_spacing * 0.5:
            msg += "  [warning: radius is large relative to spacing; most frames may label 'waypoint']"
        print(msg)
    else:
        print(f"Loaded {len(waypoints)} waypoint(s); snap disabled (--waypoint-radius-cm 0).")
    rng = random.Random(seed)
    use_jitter = jitter_xy_cm > 0.0 or jitter_yaw_deg > 0.0
    print(
        f"Loaded path ({spline.kind}): {len(spline.points)} control points, "
        f"arc length ~ {spline.total_length:.1f} UE cm"
    )
    if use_jitter:
        print(
            f"Pose jitter: xy +/-{jitter_xy_cm} cm / yaw +/-{jitter_yaw_deg} deg "
            f"(seed={seed})"
        )

    output_dir.mkdir(parents=True, exist_ok=True)
    images_dir = output_dir / "images"
    images_dir.mkdir(exist_ok=True)
    manifest_path = output_dir / "manifest.jsonl"

    agent: UnrealAgent | None = None
    resolved_cam = camera_id
    pawn_z_used = pawn_z if pawn_z is not None else 0.0

    if not dry_run:
        agent = UnrealAgent()
        print("Connecting to UnrealCV (localhost:9000) ...")
        if not agent.connect():
            print("Failed to connect to UnrealCV.", file=sys.stderr)
            return 1
        status = agent.check_status()
        print(f"Connected. Status: {status}")

        if probe_cameras:
            return _probe_cameras(agent, output_dir, debug=debug)

        if pawn_z is None:
            # Read current pawn Z so we don't drop it through the floor when
            # teleporting; fall back to 0 if UnrealCV won't tell us.
            pose = agent.get_pawn_pose()
            if pose is not None:
                pawn_z_used = pose.z
                print(f"Using current pawn Z = {pawn_z_used:.2f}")
            else:
                print("Could not read pawn Z; using 0.0")

        if capture_enabled and spawn_capture_camera:
            spawned = _spawn_capture_camera(agent, debug=debug)
            if spawned is None:
                print(
                    "Spawn-capture-camera requested but vset /cameras/spawn returned no new id; "
                    "falling back to existing cameras."
                )
            else:
                resolved_cam = spawned
                print(f"Spawned capture camera id = {resolved_cam}")

        if capture_enabled and resolved_cam is None:
            resolved_cam = resolve_lit_camera_id(agent, debug=debug)
            if resolved_cam is None:
                print("Could not resolve a lit camera id; disabling capture.")
            else:
                print(f"Lit camera id = {resolved_cam} (auto-selected)")
        elif not capture_enabled:
            resolved_cam = None
            print("Capture disabled (--no-capture).")

        # Verify the chosen camera actually responds before we start the run.
        # Without this check UE silently fails (`error Invalid sensor id`)
        # and we'd write blank/wrong frames.
        if capture_enabled and resolved_cam is not None:
            if not _is_camera_responsive(agent, resolved_cam):
                valid = _list_valid_camera_ids(agent)
                print(
                    f"ERROR: camera id {resolved_cam} is not registered with UnrealCV "
                    f"(returned 'Invalid sensor id').",
                    file=sys.stderr,
                )
                print(f"  Responsive camera ids in this session: {valid}", file=sys.stderr)
                print(
                    "  This usually happens after PIE was stopped/restarted, or if "
                    "FusionCameraActor isn't placed in the level.",
                    file=sys.stderr,
                )
                print(
                    "  Try: rerun the probe (`--probe-cameras --debug`) to see what's "
                    "available, or pass `--camera-id <id>` from that list, or place a "
                    "FusionCameraActor in the level and restart PIE.",
                    file=sys.stderr,
                )
                return 2
            print(f"Confirmed camera {resolved_cam} is responsive.")

            if camera_fov_deg > 0.0:
                if _set_camera_fov(agent, resolved_cam, camera_fov_deg, debug=debug):
                    print(f"Set camera {resolved_cam} FOV = {camera_fov_deg:.2f} deg")
                else:
                    print(
                        f"WARNING: failed to set camera {resolved_cam} FOV; "
                        f"continuing with the existing FOV.",
                        file=sys.stderr,
                    )

    _write_run_config(
        output_dir / "run_config.json",
        map_json=map_json,
        spline_length=spline.total_length,
        curve_mode=spline.kind,
        samples_per_segment=samples_per_segment,
        step_cm=step_cm,
        lookahead_cm=lookahead_cm,
        turn_threshold_deg=turn_threshold_deg,
        align_threshold_deg=align_threshold_deg,
        turn_step_deg=turn_step_deg,
        center_tol_cm=center_tol_cm,
        heading_tol_deg=heading_tol_deg,
        max_consecutive_turns=max_consecutive_turns,
        forward_step_cm=forward_step_cm,
        post_turn_forward_scale=post_turn_forward_scale,
        yaw_offset_deg=yaw_offset_deg,
        jitter_xy_cm=jitter_xy_cm,
        jitter_yaw_deg=jitter_yaw_deg,
        seed=seed,
        dry_run=dry_run,
        pawn=pawn,
        camera_id=resolved_cam,
        pawn_z=pawn_z,
        pawn_z_used=pawn_z_used,
        skip_spline_bootstrap=skip_spline_bootstrap,
        follow_spline_forward=follow_spline_forward,
        follow_spline_forward_sync_yaw=follow_spline_forward_sync_yaw,
        turn_budget_escape_heading_mult=turn_budget_escape_heading_mult,
        max_consecutive_forwards=max_consecutive_forwards,
        finish_threshold_cm=finish_threshold_cm,
        min_finish_progress_ratio=min_finish_progress_ratio,
    )

    total_len = float(spline.total_length)
    path_end = spline.sample(total_len)
    final_wp_x, final_wp_y = waypoints[-1]

    fwd_step = float(forward_step_cm) if forward_step_cm is not None else (0.5 * step_cm)
    # Policy loop: execute action, then capture image of that action result.
    # Budget is distance-first: estimate how many forward actions are needed
    # to traverse arc length, then add headroom for turn-in-place actions.
    base_steps = max(1, int(math.ceil(spline.total_length / max(step_cm, 1e-6))))
    base_forward_steps = max(1, int(math.ceil(spline.total_length / max(fwd_step, 1e-6))))
    turn_overhead = max(4, max(1, max_consecutive_turns) + 3)
    max_frames = max(
        base_steps * (max(1, max_consecutive_turns) + 1),
        base_forward_steps * turn_overhead,
    )
    print(
        f"Policy collection: base_steps={base_steps}, base_forward_steps={base_forward_steps}, "
        f"max_frames={max_frames}, "
        f"fwd_step={fwd_step:.2f} cm, turn_step={turn_step_deg:.2f} deg, "
        f"center_tol={center_tol_cm:.2f} cm, heading_tol={heading_tol_deg:.2f} deg"
    )
    if follow_spline_forward and not follow_spline_forward_sync_yaw:
        print(
            f"Split-yaw policy: after {max_consecutive_turns} turn(s), forward escape only "
            f"if |heading_err| <= {heading_tol_deg:.2f} * {turn_budget_escape_heading_mult:.2f} "
            f"(else keep turning)."
        )
        if max_consecutive_forwards is not None and max_consecutive_forwards > 0:
            print(
                f"  Forward streak cap: force turn after {max_consecutive_forwards} forward(s) "
                f"while |heading_err| > {heading_tol_deg:.2f} deg."
            )
    if finish_threshold_cm is not None:
        print(
            f"Loop-safe finish: XY within {finish_threshold_cm:.1f} cm of path end sample "
            f"AND progress_ratio >= {min_finish_progress_ratio:.2f} "
            f"(same idea as eval.py). "
            f"Arc-complete stop also requires this same end-distance gate."
        )

    # Start from the spline origin with tangent-aligned yaw. Otherwise the
    # first captures are "spawn repair": chord from PlayerStart → first sample
    # disagrees with the spline tangent at s=0 (see manifest chord_yaw vs
    # spline_yaw_deg on early rows).
    prev_x: float | None = None
    prev_y: float | None = None
    prev_yaw: float | None = None
    s_start = spline.sample(0.0)
    start_x = float(s_start.x)
    start_y = float(s_start.y)
    start_yaw = _wrap_deg(s_start.yaw_deg + yaw_offset_deg)

    if not skip_spline_bootstrap:
        if not dry_run and agent is not None:
            # Keep bootstrap atomicity explicit: rotation and location are
            # separate actions, never a single combined teleport before capture.
            set_pawn_rotation(agent, start_yaw, pawn=pawn, debug=debug)
            set_pawn_location(
                agent, start_x, start_y, z=pawn_z_used, pawn=pawn, debug=debug
            )
            if settle_s > 0:
                time.sleep(settle_s)
            print(
                f"Spline bootstrap (no capture): placed pawn at path s=0 "
                f"({start_x:.1f}, {start_y:.1f}) yaw={start_yaw:.2f} deg along tangent."
            )
        prev_x, prev_y, prev_yaw = start_x, start_y, start_yaw
    else:
        if not dry_run and agent is not None:
            cur_pose = agent.get_pawn_pose()
            if cur_pose is not None:
                prev_x = cur_pose.x
                prev_y = cur_pose.y
                prev_yaw = cur_pose.yaw
        if prev_x is None:
            prev_x, prev_y, prev_yaw = start_x, start_y, start_yaw

    frame_idx = 0
    consecutive_turns = 0
    consecutive_forwards = 0
    progress_s = 0.0
    # Arc-length index used when --follow-spline-forward advances along sample(s).
    s_along = 0.0
    if follow_spline_forward:
        if skip_spline_bootstrap:
            s_along = float(_nearest_centerline_state(spline, prev_x, prev_y)[0])
            print(
                f"Spline-follow forward: initial s_along={s_along:.2f} cm "
                f"(nearest on path from current pose)"
            )
        else:
            if follow_spline_forward_sync_yaw:
                print(
                    "Spline-follow forward: each 'forward' advances arc length on the spline "
                    "and snaps XY + yaw to the tape tangent (legacy; mixes rotate + translate)."
                )
            else:
                print(
                    "Spline-follow forward: each 'forward' advances arc length and snaps XY "
                    "to the tape only; yaw changes only on left/right steps (no rotate+forward "
                    "in one action)."
                )
    summary = {"forward": 0, "left": 0, "right": 0, "waypoint": 0}

    def _maybe_follow(x: float, y: float, yaw_deg: float) -> None:
        if not (capture_enabled and resolved_cam is not None and force_camera_follow):
            return
        cam_jx = cam_jy = cam_jyaw = 0.0
        # Randomization perturbs the CAMERA only at capture time so labels and
        # action sequencing stay deterministic (no mislabelled turn/forward).
        if randomization:
            cam_jx = rng.uniform(-DEFAULT_RANDOM_JITTER_XY_CM, DEFAULT_RANDOM_JITTER_XY_CM)
            cam_jy = rng.uniform(-DEFAULT_RANDOM_JITTER_XY_CM, DEFAULT_RANDOM_JITTER_XY_CM)
            cam_jyaw = rng.uniform(-DEFAULT_RANDOM_JITTER_YAW_DEG, DEFAULT_RANDOM_JITTER_YAW_DEG)
        yaw_rad = math.radians(yaw_deg)
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        world_dx = camera_x_offset_cm * cos_y - camera_y_offset_cm * sin_y
        world_dy = camera_x_offset_cm * sin_y + camera_y_offset_cm * cos_y
        _set_camera_pose(
            agent,
            resolved_cam,
            x=x + world_dx + cam_jx,
            y=y + world_dy + cam_jy,
            z=pawn_z_used + camera_z_offset_cm,
            pitch_deg=camera_pitch_deg,
            yaw_deg=_wrap_deg(yaw_deg + cam_jyaw),
            roll_deg=camera_roll_deg,
            debug=debug,
        )

    def _capture_and_log(
        manifest_fp,
        idx: int,
        *,
        action: str,
        label: str,
        x: float,
        y: float,
        yaw_deg: float,
        s_here: float,
        extra: dict,
    ) -> int:
        img_rel = f"images/frame_{idx:05d}_{label}.png"
        img_abs = output_dir / img_rel
        if not dry_run and agent is not None:
            if settle_s > 0:
                time.sleep(settle_s)
            _maybe_follow(x, y, yaw_deg)
            if capture_enabled and resolved_cam is not None:
                ok_cap = capture_lit_image(
                    agent, img_abs, camera_id=resolved_cam, debug=debug
                )
                if debug and not ok_cap:
                    print(f"  capture failed at s={s_here:.1f}; expected {img_abs.resolve()}")
        row = {
            "schema_version": 1,
            "frame_idx": idx,
            "distance_along_path": s_here,
            "curve_mode": spline.kind,
            "action": action,
            "label": label,
            "pose": {"x": x, "y": y, "z": pawn_z_used, "yaw_deg": yaw_deg},
            "label_detail": extra,
            "image": img_rel,
            "path_source": str(Path(map_json).resolve()),
        }
        manifest_fp.write(json.dumps(row) + "\n")
        manifest_fp.flush()
        summary[label] = summary.get(label, 0) + 1
        return idx + 1

    reached_end = False
    try:
        with open(manifest_path, "w") as manifest:
            for i in range(max_frames):
                if prev_x is None or prev_y is None or prev_yaw is None:
                    break
                ax, ay = _policy_anchor_xy(
                    prev_x,
                    prev_y,
                    prev_yaw,
                    forward_cm=policy_anchor_forward_cm,
                    right_cm=policy_anchor_right_cm,
                )
                nearest_s, cx, cy, cyaw = _nearest_centerline_state(spline, ax, ay)
                target_s = min(spline.total_length, max(progress_s + step_cm, nearest_s))
                target = spline.sample_with_lookahead(target_s, lookahead_cm)
                tx = target.x
                ty = target.y
                if use_jitter:
                    tx += rng.uniform(-jitter_xy_cm, jitter_xy_cm) if jitter_xy_cm > 0.0 else 0.0
                    ty += rng.uniform(-jitter_xy_cm, jitter_xy_cm) if jitter_xy_cm > 0.0 else 0.0
                snapped = _nearest_point_within(tx, ty, waypoints, waypoint_radius_cm)
                if snapped is not None:
                    tx, ty = snapped
                yaw_path = _wrap_deg(cyaw + yaw_offset_deg)
                heading_err = _wrap_deg(yaw_path - prev_yaw)
                # Signed cross-track error in tangent frame; + means line is left.
                yaw_rad = math.radians(yaw_path)
                nx = -math.sin(yaw_rad)
                ny = math.cos(yaw_rad)
                cross_track = (ax - cx) * nx + (ay - cy) * ny

                split_yaw_follow = follow_spline_forward and not follow_spline_forward_sync_yaw
                turn_budget_reset = False
                if snapped is not None:
                    next_label = "waypoint"
                else:
                    # Hysteresis: after turning, require a tighter "centered"
                    # band before switching back to forward, so curves favor
                    # turn->turn->forward rather than turn->forward too early.
                    center_gate = center_tol_cm
                    heading_gate = heading_tol_deg
                    if consecutive_turns > 0:
                        k = max(0.05, float(post_turn_forward_scale))
                        center_gate *= k
                        heading_gate *= k
                    centered = abs(cross_track) <= center_gate and abs(heading_err) <= heading_gate
                    forward_turn_injection = (
                        split_yaw_follow
                        and max_consecutive_forwards is not None
                        and max_consecutive_forwards > 0
                        and consecutive_forwards >= max_consecutive_forwards
                        and abs(heading_err) > heading_tol_deg
                    )
                    if forward_turn_injection:
                        next_label = _pick_turn_label(cross_track, heading_err)
                    elif centered:
                        next_label = "forward"
                    elif consecutive_turns >= max_consecutive_turns:
                        # Legacy: always escape with forward. Split-yaw: only escape to
                        # forward when roughly heading-aligned; else keep turning (refresh budget).
                        if (
                            split_yaw_follow
                            and abs(heading_err)
                            > heading_tol_deg * float(turn_budget_escape_heading_mult)
                        ):
                            turn_budget_reset = True
                            next_label = _pick_turn_label(cross_track, heading_err)
                        else:
                            next_label = "forward"
                    else:
                        next_label = _pick_turn_label(cross_track, heading_err)

                # Capture CURRENT view with label = NEXT action to take.
                frame_idx = _capture_and_log(
                    manifest,
                    frame_idx,
                    action="next_action",
                    label=next_label,
                    x=prev_x,
                    y=prev_y,
                    yaw_deg=prev_yaw,
                    s_here=nearest_s,
                    extra={
                        "nearest_s": round(nearest_s, 4),
                        "target_s": round(target_s, 4),
                        "policy_anchor": {
                            "forward_cm": policy_anchor_forward_cm,
                            "right_cm": policy_anchor_right_cm,
                            "x": round(ax, 4),
                            "y": round(ay, 4),
                        },
                        "cross_track_cm": round(cross_track, 4),
                        "heading_err_deg": round(heading_err, 4),
                        "center_tol_cm": center_tol_cm,
                        "heading_tol_deg": heading_tol_deg,
                        "center_gate_cm": center_gate if snapped is None else center_tol_cm,
                        "heading_gate_deg": heading_gate if snapped is None else heading_tol_deg,
                        "post_turn_forward_scale": post_turn_forward_scale,
                        "consecutive_turns": consecutive_turns,
                        "semantic_turn": abs(heading_err) > turn_threshold_deg,
                        "follow_spline_forward": follow_spline_forward,
                        "follow_spline_forward_sync_yaw": follow_spline_forward_sync_yaw,
                        "consecutive_forwards": consecutive_forwards,
                        "turn_budget_escape_heading_mult": turn_budget_escape_heading_mult,
                        "max_consecutive_forwards": max_consecutive_forwards,
                        "turn_budget_reset": turn_budget_reset,
                    },
                )

                # Execute the chosen action AFTER capture.
                if next_label == "left":
                    if turn_budget_reset:
                        consecutive_turns = 0
                    new_yaw = _wrap_deg(prev_yaw - turn_step_deg)
                    if not dry_run and agent is not None:
                        set_pawn_rotation(agent, new_yaw, pawn=pawn, debug=debug)
                    prev_yaw = new_yaw
                    consecutive_turns += 1
                    consecutive_forwards = 0
                elif next_label == "right":
                    if turn_budget_reset:
                        consecutive_turns = 0
                    new_yaw = _wrap_deg(prev_yaw + turn_step_deg)
                    if not dry_run and agent is not None:
                        set_pawn_rotation(agent, new_yaw, pawn=pawn, debug=debug)
                    prev_yaw = new_yaw
                    consecutive_turns += 1
                    consecutive_forwards = 0
                else:
                    # forward/waypoint: forward takes precedence over more turning.
                    step_use = math.hypot(tx - prev_x, ty - prev_y) if next_label == "waypoint" else fwd_step
                    if follow_spline_forward and next_label == "forward":
                        s_along = min(
                            float(spline.total_length),
                            s_along + float(step_use),
                        )
                        p_snap = spline.sample(s_along)
                        yaw_snap = _wrap_deg(p_snap.yaw_deg + yaw_offset_deg)
                        if not dry_run and agent is not None:
                            set_pawn_location(
                                agent,
                                float(p_snap.x),
                                float(p_snap.y),
                                z=pawn_z_used,
                                pawn=pawn,
                                debug=debug,
                            )
                            if follow_spline_forward_sync_yaw:
                                set_pawn_rotation(agent, yaw_snap, pawn=pawn, debug=debug)
                        prev_x = float(p_snap.x)
                        prev_y = float(p_snap.y)
                        if follow_spline_forward_sync_yaw:
                            prev_yaw = yaw_snap
                        progress_s = s_along
                    else:
                        fx = prev_x + step_use * math.cos(math.radians(prev_yaw))
                        fy = prev_y + step_use * math.sin(math.radians(prev_yaw))
                        if next_label == "waypoint":
                            fx, fy = tx, ty
                        if not dry_run and agent is not None:
                            set_pawn_location(agent, fx, fy, z=pawn_z_used, pawn=pawn, debug=debug)
                        prev_x, prev_y = fx, fy
                        if next_label == "waypoint" and follow_spline_forward:
                            pw = spline.sample(min(spline.total_length, float(target_s)))
                            if follow_spline_forward_sync_yaw:
                                prev_yaw = _wrap_deg(pw.yaw_deg + yaw_offset_deg)
                                if not dry_run and agent is not None:
                                    set_pawn_rotation(agent, prev_yaw, pawn=pawn, debug=debug)
                            s_along = min(float(spline.total_length), float(target_s))
                            progress_s = s_along
                        else:
                            # Match distance actually moved this frame (`step_use`), not `--step`.
                            # Default forward-step-cm is 0.5 * step; old code added step_cm here so
                            # progress_s hit total_length ~2x too early and the run stopped halfway.
                            progress_s = min(
                                spline.total_length,
                                progress_s + float(step_use),
                            )
                    consecutive_turns = 0
                    consecutive_forwards += 1

                progress_ratio = (
                    (progress_s / total_len) if total_len > 0.0 else 1.0
                )
                dist_to_end = math.hypot(prev_x - path_end.x, prev_y - path_end.y)
                dist_to_final_wp = math.hypot(prev_x - final_wp_x, prev_y - final_wp_y)
                # Never end purely on arc progress; require spatial proximity to
                # the end sample as well so loops do not stop "early enough".
                end_gate_cm = (
                    float(finish_threshold_cm)
                    if finish_threshold_cm is not None
                    else float(max(3.0, 0.5 * step_cm))
                )
                done_arc = (
                    progress_s >= total_len
                    and next_label == "forward"
                    and dist_to_end <= end_gate_cm
                    and dist_to_final_wp <= end_gate_cm
                )
                done_near_end = (
                    finish_threshold_cm is not None
                    and next_label == "forward"
                    and progress_ratio >= min_finish_progress_ratio
                    and dist_to_end <= finish_threshold_cm
                    and dist_to_final_wp <= finish_threshold_cm
                )
                if done_arc or done_near_end:
                    reached_end = True
                    break
                if i % 50 == 0 or i == max_frames - 1:
                    print(
                        f"  frame {i:04d} progress_s={progress_s:7.1f} "
                        f"pose=({prev_x:7.1f},{prev_y:7.1f}) yaw={prev_yaw:7.2f} "
                        f"next={next_label} summary={summary}"
                    )
    except KeyboardInterrupt:
        print("\nInterrupted; partial manifest written.")
    finally:
        if agent is not None:
            agent.disconnect()

    if not reached_end:
        print(
            "Run stopped before final-point gate was satisfied "
            f"(max_frames={max_frames}). Increase max_frames budget/policy turn settings.",
            file=sys.stderr,
        )
        return 3

    print(f"Done. Output: {output_dir}")
    print(f"  manifest : {manifest_path}")
    print(f"  images   : {images_dir} ({frame_idx} frames; counts={summary})")
    return 0


def _write_run_config(path: Path, **kwargs) -> None:
    cfg = {"schema_version": 1, **kwargs}
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(cfg, f, indent=2)


def _parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Drive the pawn along the spline stored in a BoxSim map JSON "
            "and collect per-frame poses (+ optional images)."
        )
    )
    p.add_argument("--map", default=DEFAULT_MAP, help=f"Path to map JSON (default: {DEFAULT_MAP})")
    p.add_argument("--run-name", default="run", help="Subfolder name under data/datasets/")
    p.add_argument("--output-root", default=DEFAULT_OUTPUT_ROOT)
    p.add_argument("--step", type=float, default=DEFAULT_STEP_CM, help="Arc-length step (UE cm)")
    p.add_argument("--lookahead", type=float, default=DEFAULT_LOOKAHEAD_CM,
                   help="Arc-length look-ahead used for yaw + turn label (UE cm)")
    p.add_argument("--settle", type=float, default=DEFAULT_SETTLE_S,
                   help="Seconds to sleep after each teleport, before capture")
    p.add_argument("--turn-threshold", type=float, default=DEFAULT_TURN_THRESHOLD_DEG,
                   help=(
                       "Deg: if |delta-yaw| exceeds this on a rotation frame, "
                       "manifest marks semantic_turn=true (sharp corner vs gentle align)."
                   ))
    p.add_argument(
        "--align-threshold",
        type=float,
        default=DEFAULT_ALIGN_THRESHOLD_DEG,
        help=(
            "Legacy threshold retained for compatibility; policy now uses "
            "--center-tol-cm/--heading-tol-deg plus --max-consecutive-turns."
        ),
    )
    p.add_argument(
        "--turn-step-deg",
        type=float,
        default=DEFAULT_TURN_STEP_DEG,
        help="Per-action yaw increment for left/right decisions (default: 2.5 deg).",
    )
    p.add_argument(
        "--center-tol-cm",
        type=float,
        default=DEFAULT_CENTER_TOL_CM,
        help="If |cross-track error| <= this, prefer forward (default: 6 cm).",
    )
    p.add_argument(
        "--heading-tol-deg",
        type=float,
        default=DEFAULT_HEADING_TOL_DEG,
        help="If |heading error| <= this, prefer forward (default: 4 deg).",
    )
    p.add_argument(
        "--max-consecutive-turns",
        type=int,
        default=DEFAULT_MAX_CONSEC_TURNS,
        help="Force one forward after this many turns in a row (default: 2).",
    )
    p.add_argument(
        "--turn-budget-escape-heading-mult",
        type=float,
        default=DEFAULT_TURN_BUDGET_ESCAPE_HEADING_MULT,
        help=(
            "Only with --follow-spline-forward and without --follow-spline-forward-sync-yaw: "
            "after --max-consecutive-turns, take forward only if |heading_err| <= "
            "heading_tol * this factor; otherwise keep turning (default: 2.5)."
        ),
    )
    p.add_argument(
        "--max-consecutive-forwards",
        type=int,
        default=None,
        metavar="N",
        help=(
            "Only with split-yaw spline follow: after N consecutive forward/waypoint "
            "steps, force left/right if |heading_err| still exceeds --heading-tol-deg "
            "(curved-track diversity). Default: unset (off)."
        ),
    )
    p.add_argument(
        "--forward-step-cm",
        type=float,
        default=None,
        help=(
            "Forward action distance in cm (default: 0.5 * --step)."
        ),
    )
    p.add_argument(
        "--post-turn-forward-scale",
        type=float,
        default=DEFAULT_POST_TURN_FORWARD_SCALE,
        help=(
            "After one or more consecutive turns, scale the forward-center "
            "gates by this factor before allowing forward (default: 0.5). "
            "Lower = keep turning longer on curves."
        ),
    )
    p.add_argument("--pawn", default=None, help="Pawn object name (else UNREALCV_PAWN)")
    p.add_argument("--pawn-z", type=float, default=None,
                   help="World Z (cm) for the pawn; defaults to pawn's current Z")
    p.add_argument("--camera-id", type=int, default=None, help="Lit camera id for capture")
    p.add_argument("--no-capture", action="store_true",
                   help="Skip PNG capture (still teleports and writes manifest)")
    p.add_argument("--dry-run", action="store_true",
                   help="Do not connect to UnrealCV; just write manifest from the spline")
    p.add_argument("--yaw-offset", type=float, default=0.0,
                   help="Degrees added to spline tangent yaw before teleport/logging")
    p.add_argument(
        "--curve",
        choices=("catmull_rom", "polyline"),
        default="catmull_rom",
        help=(
            "catmull_rom: smooth curve through JSON points (closer to UE spline tape); "
            "polyline: straight chords only (legacy)."
        ),
    )
    p.add_argument(
        "--curve-samples",
        type=int,
        default=DEFAULT_CURVE_SAMPLES_PER_SEGMENT,
        help="Polyline resolution per control segment for arc length (catmull_rom only).",
    )
    p.add_argument(
        "--jitter-xy",
        type=float,
        default=0.0,
        help="Uniform random offset on X and Y each in [-value, +value] cm (0 = off).",
    )
    p.add_argument(
        "--jitter-yaw",
        type=float,
        default=0.0,
        help="Uniform random yaw offset in [-value, +value] degrees (0 = off).",
    )
    p.add_argument(
        "--seed",
        type=int,
        default=None,
        help="RNG seed for jitter (default: nondeterministic).",
    )
    p.add_argument(
        "--randomization",
        action="store_true",
        help=(
            "Enable slight camera-only capture perturbation so repeated "
            "collections differ a bit, without changing turn/forward labels. "
            f"Uses +/-{DEFAULT_RANDOM_JITTER_XY_CM} cm XY and "
            f"+/-{DEFAULT_RANDOM_JITTER_YAW_DEG} deg yaw on the followed camera."
        ),
    )
    p.add_argument(
        "--num-runs",
        type=int,
        default=1,
        help=(
            "Number of collection runs to execute. For num-runs > 1, output "
            "folders are suffixed _r001, _r002, ..."
        ),
    )
    p.add_argument(
        "--probe-cameras",
        action="store_true",
        help=(
            "Debug camera selection: print objects/cameras, capture one image per "
            "camera, and exit."
        ),
    )
    p.add_argument(
        "--force-camera-follow",
        action="store_true",
        help=(
            "Before each capture, force the selected UnrealCV camera pose to "
            "the pawn XY/yaw."
        ),
    )
    p.add_argument(
        "--camera-x-offset-cm",
        type=float,
        default=0.0,
        help=(
            "Pawn-local forward offset for the followed camera "
            "(positive = ahead of pawn). Default: 0.0 cm."
        ),
    )
    p.add_argument(
        "--camera-y-offset-cm",
        type=float,
        default=0.0,
        help=(
            "Pawn-local right offset for the followed camera. "
            "Default: 0.0 cm."
        ),
    )
    p.add_argument(
        "--camera-z-offset-cm",
        type=float,
        default=0.0,
        help=(
            "World Z offset (above pawn) for the followed camera. "
            "Default: 0.0 cm."
        ),
    )
    p.add_argument(
        "--camera-pitch-deg",
        type=float,
        default=0.0,
        help=(
            "Camera pitch in degrees (UnrealCV order pitch yaw roll). "
            "More negative = look further toward the ground. Default: 0.0."
        ),
    )
    p.add_argument(
        "--camera-roll-deg",
        type=float,
        default=0.0,
        help="Camera roll in degrees. Default: 0.0.",
    )
    p.add_argument(
        "--camera-fov-deg",
        type=float,
        default=0.0,
        help=(
            "Override camera FOV in degrees (applied once at startup). "
            "Default: 0.0 = leave UnrealCV's current FOV unchanged "
            "(typically 90 deg)."
        ),
    )
    p.add_argument(
        "--spawn-capture-camera",
        action="store_true",
        help=(
            "Call `vset /cameras/spawn` at startup and use the new camera id "
            "for captures. Useful when camera 0 is locked to the player "
            "controller and ignores vset."
        ),
    )
    p.add_argument(
        "--waypoint-radius-cm",
        type=float,
        default=DEFAULT_WAYPOINT_RADIUS_CM,
        help=(
            "When the spline target is within this distance of any path "
            "waypoint (spline control point), snap teleport to the exact "
            "waypoint and label the frame 'waypoint'. "
            f"Default: {DEFAULT_WAYPOINT_RADIUS_CM} cm. Set to 0 to disable."
        ),
    )
    p.add_argument(
        "--skip-spline-bootstrap",
        action="store_true",
        help=(
            "Do not teleport to spline start (s=0) with tangent yaw before "
            "captures; keep the pawn where it spawned (first frames correct "
            "spawn->path). Default: bootstrap so dataset aligns with the tape "
            "from frame 0."
        ),
    )
    p.add_argument(
        "--policy-anchor-forward-cm",
        type=float,
        default=0.0,
        help=(
            "Centerline math uses pawn XY plus this offset along pawn forward (+ = ahead). "
            "Does not move the pawn—only labels / cross-track vs spline. Default: 0."
        ),
    )
    p.add_argument(
        "--policy-anchor-right-cm",
        type=float,
        default=0.0,
        help=(
            "Same as forward, along pawn right (+ = actor's right). "
            "If the mesh pivot sits on the left side of the body, try a positive "
            "value (centimeters) so the tape aligns with the visual center of the agent."
        ),
    )
    p.add_argument(
        "--follow-spline-forward",
        action="store_true",
        help=(
            "On each 'forward' action, advance arc length along the spline and snap pawn "
            "XY to the tape sample (fixes drift vs stepping along vehicle heading). "
            "By default yaw is not updated on forward — only left/right rotate in place — "
            "so turns and translation are not combined in one step and labels are not "
            "dominated by 'forward'. Use --follow-spline-forward-sync-yaw for legacy "
            "XY+tangent-yaw on every forward. Default: off (heading-based forward)."
        ),
    )
    p.add_argument(
        "--follow-spline-forward-sync-yaw",
        action="store_true",
        help=(
            "Only with --follow-spline-forward: also set pawn yaw to the spline tangent on "
            "each forward/waypoint step (same timestep as XY). Without this flag, forward "
            "only translates along the arc; heading changes only on explicit turn actions."
        ),
    )
    p.add_argument(
        "--finish-threshold-cm",
        type=float,
        default=None,
        metavar="CM",
        help=(
            "Optional loop-safe stop: end collection when pawn XY is within this distance "
            "(cm) of the path end point AND progress_ratio >= --min-finish-progress-ratio "
            "(eval-style gate so closed tracks do not finish near start). "
            "Default: unset (only stop when arc progress reaches full path length)."
        ),
    )
    p.add_argument(
        "--min-finish-progress-ratio",
        type=float,
        default=0.80,
        help=(
            "Gate for --finish-threshold-cm (ignored if threshold unset). "
            "Ignore proximity-to-end until progress / path length reaches this ratio."
        ),
    )
    p.add_argument("--debug", action="store_true")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(list(sys.argv[1:] if argv is None else argv))
    if args.num_runs < 1:
        print("--num-runs must be >= 1", file=sys.stderr)
        return 2

    jitter_xy = args.jitter_xy
    jitter_yaw = args.jitter_yaw

    base_seed = args.seed
    root = Path(args.output_root)
    rc = 0
    for run_idx in range(args.num_runs):
        run_name = (
            args.run_name
            if args.num_runs == 1
            else f"{args.run_name}_r{run_idx + 1:03d}"
        )
        output_dir = root / run_name

        seed_for_run = base_seed
        if args.num_runs > 1:
            if base_seed is None:
                seed_for_run = random.randint(0, 2_147_483_647)
            else:
                seed_for_run = int(base_seed) + run_idx

        print(
            f"\n=== Collect run {run_idx + 1}/{args.num_runs}: {run_name} "
            f"(seed={seed_for_run}, jitter_xy={jitter_xy}, jitter_yaw={jitter_yaw}) ==="
        )
        rc = run(
            map_json=args.map,
            output_dir=output_dir,
            step_cm=args.step,
            lookahead_cm=args.lookahead,
            settle_s=args.settle,
            turn_threshold_deg=args.turn_threshold,
            align_threshold_deg=args.align_threshold,
            turn_step_deg=args.turn_step_deg,
            center_tol_cm=args.center_tol_cm,
            heading_tol_deg=args.heading_tol_deg,
            max_consecutive_turns=args.max_consecutive_turns,
            forward_step_cm=args.forward_step_cm,
            post_turn_forward_scale=args.post_turn_forward_scale,
            pawn=args.pawn,
            pawn_z=args.pawn_z,
            camera_id=args.camera_id,
            capture_enabled=(not args.no_capture) and (not args.dry_run),
            dry_run=args.dry_run,
            debug=args.debug,
            yaw_offset_deg=args.yaw_offset,
            curve=cast(CurveMode, args.curve),
            samples_per_segment=args.curve_samples,
            jitter_xy_cm=jitter_xy,
            jitter_yaw_deg=jitter_yaw,
            seed=seed_for_run,
            probe_cameras=args.probe_cameras,
            force_camera_follow=args.force_camera_follow,
            camera_x_offset_cm=args.camera_x_offset_cm,
            camera_y_offset_cm=args.camera_y_offset_cm,
            camera_z_offset_cm=args.camera_z_offset_cm,
            camera_pitch_deg=args.camera_pitch_deg,
            camera_roll_deg=args.camera_roll_deg,
            camera_fov_deg=args.camera_fov_deg,
            spawn_capture_camera=args.spawn_capture_camera,
            waypoint_radius_cm=args.waypoint_radius_cm,
            skip_spline_bootstrap=args.skip_spline_bootstrap,
            randomization=args.randomization,
            policy_anchor_forward_cm=args.policy_anchor_forward_cm,
            policy_anchor_right_cm=args.policy_anchor_right_cm,
            follow_spline_forward=args.follow_spline_forward,
            follow_spline_forward_sync_yaw=args.follow_spline_forward_sync_yaw,
            turn_budget_escape_heading_mult=args.turn_budget_escape_heading_mult,
            max_consecutive_forwards=args.max_consecutive_forwards,
            finish_threshold_cm=args.finish_threshold_cm,
            min_finish_progress_ratio=args.min_finish_progress_ratio,
        )
        if rc != 0:
            return rc
    return rc


if __name__ == "__main__":
    sys.exit(main())
