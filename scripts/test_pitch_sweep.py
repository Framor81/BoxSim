#!/usr/bin/env python3
"""Teleport to spline start (like collect frame 0) and capture one lit image per pitch.

Pitch sweep: 20, 15, …, 0, …, -30 (step 5 deg). Saves under ``test_pitch/`` by default.

If you pass ``outline_trace4.json`` (inch trace), the script uses sibling ``outline_trace4_w.json``
for world-space spline sampling (same as collect).

Example::

    python scripts/test_pitch_sweep.py --map data/maps/outline_trace4.json
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import cast

_THIS = Path(__file__).resolve()
_REPO = _THIS.parent.parent
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from agent import UnrealAgent  # noqa: E402
from data_collection.capture import capture_lit_image, resolve_lit_camera_id  # noqa: E402
from data_collection.pawn_control import teleport_pawn  # noqa: E402
from data_collection.spline import CurveMode, load_path_from_map  # noqa: E402


def _resolve_world_map_json(map_path: Path) -> Path:
    """UE placement uses world cm; inch ``outline_traceN.json`` -> ``outline_traceN_w.json``."""
    p = map_path.resolve()
    if p.name.endswith("_w.json"):
        return p
    if p.is_file() and p.suffix == ".json" and not p.stem.endswith("_w"):
        cand = p.with_name(p.stem + "_w.json")
        if cand.is_file():
            print(f"Note: using world map {cand.name} for positions ({p.name} is inch-relative).")
            return cand
    return p


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
    cmd_loc = f"vset /camera/{camera_id}/location {x:.4f} {y:.4f} {z:.4f}"
    cmd_rot = f"vset /camera/{camera_id}/rotation {pitch_deg:.4f} {yaw_deg:.4f} {roll_deg:.4f}"
    if debug:
        print(f"  >> {cmd_loc}")
        print(f"  >> {cmd_rot}")
    agent.client.request(cmd_loc)
    agent.client.request(cmd_rot)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--map",
        type=Path,
        default=_REPO / "data/maps/outline_trace4.json",
        help="Map JSON (inch ok; world *_w.json used automatically if present).",
    )
    p.add_argument(
        "--output-dir",
        type=Path,
        default=_REPO / "test_pitch",
        help="Folder for PNGs (created if missing).",
    )
    p.add_argument("--curve-samples", type=int, default=48)
    p.add_argument("--pawn", default=None)
    p.add_argument("--pawn-z", type=float, default=None)
    p.add_argument("--camera-id", type=int, default=None)
    p.add_argument("--spawn-capture-camera", action="store_true")
    p.add_argument("--settle", type=float, default=0.12)
    p.add_argument("--camera-x-offset-cm", type=float, default=-2.0)
    p.add_argument("--camera-y-offset-cm", type=float, default=0.0)
    p.add_argument("--camera-z-offset-cm", type=float, default=14.846071)
    p.add_argument("--camera-roll-deg", type=float, default=0.0)
    p.add_argument("--camera-fov-deg", type=float, default=100.0)
    p.add_argument("--debug", action="store_true")
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    world_map = _resolve_world_map_json(args.map)
    if not world_map.is_file():
        print(f"Map not found: {world_map}", file=sys.stderr)
        return 2

    spline = load_path_from_map(
        world_map,
        curve=cast(CurveMode, "catmull_rom"),
        samples_per_segment=args.curve_samples,
    )
    start = spline.sample(0.0)
    start_yaw = (start.yaw_deg + 180.0) % 360.0 - 180.0

    pitches = list(range(20, -31, -5))
    out_dir = args.output_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    agent = UnrealAgent()
    if not agent.connect():
        print("Failed to connect to UnrealCV.", file=sys.stderr)
        return 2

    try:
        cam_id = args.camera_id
        if args.spawn_capture_camera:
            try:
                agent.client.request("vset /cameras/spawn")
            except Exception as e:
                if args.debug:
                    print(f"spawn camera: {e}")
        if cam_id is None:
            cam_id = resolve_lit_camera_id(agent, debug=args.debug)
        if cam_id is None:
            print("Could not resolve camera id.", file=sys.stderr)
            return 2

        pawn_z = args.pawn_z
        if pawn_z is None:
            pose0 = agent.get_pawn_pose()
            pawn_z = float(pose0.z) if pose0 is not None else 0.0

        teleport_pawn(
            agent,
            start.x,
            start.y,
            start_yaw,
            z=float(pawn_z),
            pawn=args.pawn,
            debug=args.debug,
        )
        if args.settle > 0:
            time.sleep(args.settle)

        if args.camera_fov_deg > 0:
            try:
                agent.client.request(f"vset /camera/{cam_id}/fov {args.camera_fov_deg:.4f}")
            except Exception:
                pass

        yaw_rad = math.radians(start_yaw)
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        world_dx = args.camera_x_offset_cm * cos_y - args.camera_y_offset_cm * sin_y
        world_dy = args.camera_x_offset_cm * sin_y + args.camera_y_offset_cm * cos_y
        cx = start.x + world_dx
        cy = start.y + world_dy
        cz = float(pawn_z) + args.camera_z_offset_cm

        print(f"Spline s=0 pose x={start.x:.2f} y={start.y:.2f} yaw={start_yaw:.2f}")
        print(f"Saving {len(pitches)} frames to {out_dir}")

        for pitch in pitches:
            _set_camera_pose(
                agent,
                cam_id,
                x=cx,
                y=cy,
                z=cz,
                pitch_deg=float(pitch),
                yaw_deg=start_yaw,
                roll_deg=args.camera_roll_deg,
                debug=args.debug,
            )
            if args.settle > 0:
                time.sleep(args.settle)
            tag = f"{pitch:+d}".replace("+", "p").replace("-", "m")
            img_path = out_dir / f"pitch_{tag}.png"
            ok = capture_lit_image(agent, img_path, camera_id=cam_id, debug=args.debug)
            if not ok:
                print(f"Capture failed for pitch {pitch} -> {img_path}", file=sys.stderr)
                return 2
            print(f"  wrote {img_path.name}")

        print("Done.")
        return 0
    finally:
        agent.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
