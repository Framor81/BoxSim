#!/usr/bin/env python3
"""Merge inch trace JSON + world export into one ``outline_traceN_w.json`` (same shape as trace4).

**Easy usage** (from repo root)::

    python scripts/normalize_map_json.py 1

That reads ``data/maps/outline_trace1.json`` + ``data/maps/outline_trace1_w.json`` and
writes the merged result back to ``outline_trace1_w.json``.

**Explicit paths** (only if you need them)::

    python scripts/normalize_map_json.py --world path/to/outline_trace1_w.json

Plan file is the sibling ``outline_trace1.json`` unless you pass ``--plan``.
"""

from __future__ import annotations

import argparse
import copy
import json
import re
import sys
from pathlib import Path


def _repo_maps_dir() -> Path:
    return Path(__file__).resolve().parent.parent / "data" / "maps"


def _trace_base(spec: str) -> str:
    spec = spec.strip()
    if not spec:
        raise ValueError("empty trace id")
    if re.fullmatch(r"\d+", spec):
        return f"outline_trace{spec}"
    if spec.startswith("outline_trace"):
        return spec
    raise ValueError(
        f"trace id {spec!r} — use a number (e.g. 1) or outline_trace1"
    )


def _infer_plan_from_world(world_path: Path) -> Path:
    stem = world_path.stem
    if not stem.endswith("_w"):
        raise ValueError(f"expected world file named like *_w.json, got {world_path.name}")
    base = stem[:-2]
    return world_path.with_name(base + ".json")


def _path_xy_from_world_points(meta: dict) -> list[list[float]] | None:
    wp = meta.get("world_points")
    if not isinstance(wp, list) or len(wp) < 2:
        return None
    rows: list[tuple[int, float, float]] = []
    for i, item in enumerate(wp):
        if isinstance(item, dict) and "x" in item and "y" in item:
            idx = int(item["index"]) if item.get("index") is not None else i
            rows.append((idx, float(item["x"]), float(item["y"])))
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            rows.append((i, float(item[0]), float(item[1])))
    if len(rows) < 2:
        return None
    rows.sort(key=lambda t: t[0])
    return [[x, y] for _, x, y in rows]


def _world_path_from_export(world_meta: dict) -> list[list[float]] | None:
    raw = world_meta.get("path")
    if isinstance(raw, list) and len(raw) >= 2:
        out: list[list[float]] = []
        for p in raw:
            if isinstance(p, (list, tuple)) and len(p) >= 2:
                out.append([float(p[0]), float(p[1])])
            elif isinstance(p, dict) and "x" in p and "y" in p:
                out.append([float(p["x"]), float(p["y"])])
        if len(out) >= 2:
            return out
    return _path_xy_from_world_points(world_meta)


def _z_hint_from_world_points(world_meta: dict) -> float | None:
    wp = world_meta.get("world_points")
    if not isinstance(wp, list) or not wp:
        return None
    zs: list[float] = []
    for item in wp:
        if isinstance(item, dict) and "z" in item:
            try:
                zs.append(float(item["z"]))
            except (TypeError, ValueError):
                continue
    if not zs:
        return None
    return sum(zs) / len(zs)


def merge_plan_and_world(plan: dict, world_export: dict) -> dict:
    inch_path = plan.get("path")
    if not isinstance(inch_path, list) or len(inch_path) < 2:
        raise ValueError("plan JSON must contain a 'path' with at least 2 points")

    world_path_pts = _world_path_from_export(world_export)
    if not world_path_pts:
        raise ValueError("world export must contain 'path' or usable 'world_points'")

    inch_fmt = [[float(p[0]), float(p[1])] for p in inch_path if len(p) >= 2]
    if len(inch_fmt) != len(world_path_pts):
        raise ValueError(
            f"plan path length ({len(inch_fmt)}) != world path length ({len(world_path_pts)})"
        )

    out: dict = {}
    for key, val in plan.items():
        if key == "path":
            out["path"] = world_path_pts
        else:
            out[key] = copy.deepcopy(val)

    out["path_relative_in"] = inch_fmt
    out["path_space"] = "world"
    out["path_world_point_count"] = len(world_path_pts)

    zh = _z_hint_from_world_points(world_export)
    if zh is not None:
        out["world_point_z_hint"] = zh

    src = world_export.get("path_world_source")
    if isinstance(src, str) and src.strip():
        out["path_world_source"] = src

    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "trace",
        nargs="?",
        default=None,
        help="Short id: 1 / 4 / outline_trace1 — merges data/maps/outline_traceN.json + *_w.json",
    )
    ap.add_argument(
        "--maps-dir",
        type=Path,
        default=None,
        help=f"Default: {_repo_maps_dir()}",
    )
    ap.add_argument(
        "--plan",
        type=Path,
        default=None,
        help="Override inch JSON path",
    )
    ap.add_argument(
        "--world",
        type=Path,
        default=None,
        help="Override world JSON path (default: sibling outline_traceN_w.json)",
    )
    ap.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output path (default: overwrite --world file)",
    )
    args = ap.parse_args()

    maps_dir = args.maps_dir or _repo_maps_dir()

    plan_path: Path | None = args.plan
    world_path: Path | None = args.world

    if args.trace:
        base = _trace_base(args.trace)
        plan_path = maps_dir / f"{base}.json"
        world_path = maps_dir / f"{base}_w.json"
    elif world_path is None:
        ap.print_help()
        print(
            "\nGive a trace id, for example:\n  python scripts/normalize_map_json.py 1\n",
            file=sys.stderr,
        )
        return 2

    if plan_path is None:
        plan_path = _infer_plan_from_world(world_path)

    assert world_path is not None

    if not plan_path.is_file():
        print(f"Not found (inch trace): {plan_path}", file=sys.stderr)
        return 2
    if not world_path.is_file():
        print(f"Not found (world export): {world_path}", file=sys.stderr)
        return 2

    plan_meta = json.loads(plan_path.read_text(encoding="utf-8"))
    world_meta = json.loads(world_path.read_text(encoding="utf-8"))

    try:
        meta = merge_plan_and_world(plan_meta, world_meta)
    except ValueError as e:
        print(str(e), file=sys.stderr)
        return 2

    out = args.output or world_path
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(f"Merged\n  plan : {plan_path}\n  world: {world_path}\n  ->   {out.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
