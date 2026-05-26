#!/usr/bin/env python3
"""Insert extra vertices along long polyline edges so Catmull-Rom sampling stays stable.

Long spans (from sparse clicks) plus tight corners cause spline overshoot / twists.
Default: ensure no segment exceeds ``--max-seg-in`` inches on inch-relative maps.

Example::

    python scripts/densify_map_path.py data/maps/outline_trace3.json -o data/maps/outline_trace3.json --in-place-backup
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import shutil
import sys
from pathlib import Path


def _dist(a: list[float], b: list[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def densify_path(
    path: list[list[float]], *, max_seg: float
) -> tuple[list[list[float]], int]:
    if len(path) < 2:
        return path, 0
    out: list[list[float]] = [path[0][:2]]
    inserted = 0
    for i in range(len(path) - 1):
        a = path[i]
        b = path[i + 1]
        d = _dist(a, b)
        if d <= max_seg or d < 1e-9:
            out.append([float(b[0]), float(b[1])])
            continue
        n_extra = int(math.ceil(d / max_seg)) - 1
        for k in range(1, n_extra + 1):
            t = k / (n_extra + 1)
            out.append(
                [
                    float(a[0]) + t * (float(b[0]) - float(a[0])),
                    float(a[1]) + t * (float(b[1]) - float(a[1])),
                ]
            )
            inserted += 1
        out.append([float(b[0]), float(b[1])])
    return out, inserted


def _n_interior_points(edge_len: float, max_seg: float) -> int:
    if edge_len <= max_seg or edge_len < 1e-9 or max_seg <= 0:
        return 0
    return int(math.ceil(edge_len / max_seg)) - 1


def densify_parallel(
    primary: list[list[float]],
    secondary: list[list[float]],
    *,
    max_seg_primary: float,
    max_seg_secondary: float | None = None,
) -> tuple[list[list[float]], list[list[float]], int]:
    """Split long edges using max interior points needed for primary and optional secondary.

    ``primary`` is usually inch trace (``path_relative_in``); ``secondary`` is world ``path``.
    For each edge, ``n_extra = max(n_in, n_world)`` so both spaces stay aligned.
    """
    if len(primary) != len(secondary):
        raise ValueError("parallel paths must have equal length")
    if len(primary) < 2:
        return primary, secondary, 0
    out_p: list[list[float]] = [primary[0][:2]]
    out_s: list[list[float]] = [secondary[0][:2]]
    inserted = 0
    for i in range(len(primary) - 1):
        a_p, b_p = primary[i], primary[i + 1]
        a_s, b_s = secondary[i], secondary[i + 1]
        d_p = _dist(a_p, b_p)
        d_s = _dist(a_s, b_s)
        n_in = _n_interior_points(d_p, max_seg_primary)
        n_world = (
            _n_interior_points(d_s, max_seg_secondary)
            if max_seg_secondary is not None and max_seg_secondary > 0
            else 0
        )
        n_extra = max(n_in, n_world)
        if n_extra <= 0:
            out_p.append([float(b_p[0]), float(b_p[1])])
            out_s.append([float(b_s[0]), float(b_s[1])])
            continue
        for k in range(1, n_extra + 1):
            t = k / (n_extra + 1)
            out_p.append(
                [
                    float(a_p[0]) + t * (float(b_p[0]) - float(a_p[0])),
                    float(a_p[1]) + t * (float(b_p[1]) - float(a_p[1])),
                ]
            )
            out_s.append(
                [
                    float(a_s[0]) + t * (float(b_s[0]) - float(a_s[0])),
                    float(a_s[1]) + t * (float(b_s[1]) - float(a_s[1])),
                ]
            )
            inserted += 1
        out_p.append([float(b_p[0]), float(b_p[1])])
        out_s.append([float(b_s[0]), float(b_s[1])])
    return out_p, out_s, inserted


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input_json", type=Path)
    ap.add_argument("-o", "--output", type=Path, default=None)
    ap.add_argument(
        "--max-seg-in",
        type=float,
        default=6.0,
        help="Max polyline segment length in map inches before inserting points.",
    )
    ap.add_argument(
        "--max-seg-cm",
        type=float,
        default=None,
        help="When path and path_relative_in are both present, also split edges longer "
        "than this many cm in world path (uses max of inch and cm criteria per edge).",
    )
    ap.add_argument(
        "--in-place-backup",
        action="store_true",
        help="If writing over input, save copy as <name>.bak.json first.",
    )
    args = ap.parse_args()

    inp = args.input_json.resolve()
    if not inp.is_file():
        print(f"Not found: {inp}", file=sys.stderr)
        return 2

    meta = json.loads(inp.read_text(encoding="utf-8"))
    path = meta.get("path")
    if not isinstance(path, list) or len(path) < 2:
        print("No path array with 2+ points.", file=sys.stderr)
        return 2

    path_in = meta.get("path_relative_in")
    meta2 = copy.deepcopy(meta)

    if (
        isinstance(path_in, list)
        and len(path_in) == len(path)
        and len(path_in) >= 2
    ):
        new_in, new_world, n_ins = densify_parallel(
            path_in,
            path,
            max_seg_primary=args.max_seg_in,
            max_seg_secondary=args.max_seg_cm,
        )
        meta2["path_relative_in"] = new_in
        meta2["path"] = new_world
        meta2["densify_mode"] = "parallel_inch_world"
        n_before = len(path)
        n_after = len(new_world)
    else:
        new_world, n_ins = densify_path(path, max_seg=args.max_seg_in)
        meta2["path"] = new_world
        meta2["densify_mode"] = "path_only"
        n_before = len(path)
        n_after = len(new_world)

    meta2["densify_max_seg_in"] = args.max_seg_in
    if args.max_seg_cm is not None:
        meta2["densify_max_seg_cm"] = args.max_seg_cm
    meta2["densify_inserted_points"] = n_ins
    if "path_world_point_count" in meta2:
        meta2["path_world_point_count"] = n_after

    out = args.output if args.output is not None else inp.with_name(inp.stem + "_densified.json")
    out = out.resolve()
    if args.in_place_backup and out == inp:
        bak = inp.with_suffix(".bak.json")
        shutil.copy2(inp, bak)
        print(f"Backup: {bak}")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(meta2, indent=2), encoding="utf-8")
    print(
        f"Wrote {out} ({n_before} -> {n_after} points, inserted {n_ins} vertices, "
        f"max_seg={args.max_seg_in} in)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
