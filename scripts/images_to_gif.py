#!/usr/bin/env python3
"""Build an animated GIF from all PNGs in a folder (sorted by filename).

Example::

    python scripts/images_to_gif.py --input data/datasets/track1_r001/images \\
        --output data/datasets/track1_r001/track1_r001.gif
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--input",
        "-i",
        type=Path,
        default=Path("data/datasets/track1_r001/images"),
        help="Folder containing *.png frames",
    )
    ap.add_argument(
        "--output",
        "-o",
        type=Path,
        default=None,
        help="Output .gif path (default: <input_parent>/<run_name>.gif)",
    )
    ap.add_argument(
        "--duration-ms",
        type=int,
        default=80,
        help="Milliseconds per frame (default: 80)",
    )
    ap.add_argument(
        "--loop",
        type=int,
        default=0,
        help="Loop count; 0 = infinite (default)",
    )
    args = ap.parse_args()

    inp = args.input.resolve()
    if not inp.is_dir():
        print(f"Not a directory: {inp}", file=sys.stderr)
        return 2

    frames = sorted(inp.glob("*.png"))
    if not frames:
        print(f"No PNG files in {inp}", file=sys.stderr)
        return 2

    try:
        from PIL import Image
    except ImportError:
        print(
            "Need Pillow: pip install pillow\n"
            "(often already installed with torchvision/fastai.)",
            file=sys.stderr,
        )
        return 2

    out = args.output
    if out is None:
        run_dir = inp.parent
        name = run_dir.name if run_dir.name else "animation"
        out = run_dir / f"{name}.gif"
    else:
        out = args.output.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)

    loaded: list[Image.Image] = []
    base_size: tuple[int, int] | None = None
    for p in frames:
        im = Image.open(p).convert("RGB")
        if base_size is None:
            base_size = im.size
        elif im.size != base_size:
            im = im.resize(base_size, Image.Resampling.LANCZOS)
        loaded.append(im)

    first, *rest = loaded
    first.save(
        out,
        save_all=True,
        append_images=rest,
        duration=args.duration_ms,
        loop=args.loop,
        optimize=False,
    )
    print(f"Wrote {out} ({len(loaded)} frames, {args.duration_ms} ms/frame)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
