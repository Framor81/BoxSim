#!/usr/bin/env python3
"""Print a paste-ready ``\"path\": [ ... ]`` block from a map / UE export JSON.

Reads ``path`` or ``world_points`` (sorted by ``index``). No merging, no metadata.

Example::

    python scripts/emit_path_snippet.py data/maps/outline_trace1_w.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def _xy_from_meta(meta: dict) -> list[list[float]]:
    raw = meta.get("path")
    if isinstance(raw, list) and len(raw) >= 2:
        out: list[list[float]] = []
        for p in raw:
            if isinstance(p, (list, tuple)) and len(p) >= 2:
                out.append([float(p[0]), float(p[1])])
            elif isinstance(p, dict) and "x" in p and "y" in p:
                out.append([float(p["x"]), float(p["y"])])
        if len(out) >= 2:
            return out

    wp = meta.get("world_points")
    if not isinstance(wp, list) or len(wp) < 2:
        raise ValueError("need 'path' or 'world_points' with x/y")
    rows: list[tuple[int, float, float]] = []
    for i, item in enumerate(wp):
        if isinstance(item, dict) and "x" in item and "y" in item:
            idx = int(item["index"]) if item.get("index") is not None else i
            rows.append((idx, float(item["x"]), float(item["y"])))
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            rows.append((i, float(item[0]), float(item[1])))
    if len(rows) < 2:
        raise ValueError("not enough world_points")
    rows.sort(key=lambda t: t[0])
    return [[x, y] for _, x, y in rows]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "json_file",
        type=Path,
        nargs="?",
        default=None,
        help="Map or export JSON (default: read stdin)",
    )
    args = ap.parse_args()

    if args.json_file is not None:
        text = args.json_file.read_text(encoding="utf-8")
    else:
        text = sys.stdin.read()

    meta = json.loads(text)
    try:
        pts = _xy_from_meta(meta)
    except ValueError as e:
        print(str(e), file=sys.stderr)
        return 2

    lines = ['"path": [']
    for i, pair in enumerate(pts):
        comma = "," if i < len(pts) - 1 else ""
        inner = json.dumps(pair)
        lines.append(f"    {inner}{comma}")
    lines.append("]")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
