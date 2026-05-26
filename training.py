from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
from typing import Iterable


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Train a simple left/right/forward image classifier from "
            "collected BoxSim dataset images."
        )
    )
    p.add_argument(
        "--data-root",
        default="data/datasets",
        help="Root folder containing run subfolders (default: data/datasets).",
    )
    p.add_argument(
        "--run-glob",
        default="run*",
        help="Run folder glob under --data-root (default: run*).",
    )
    p.add_argument(
        "--classes",
        default="left,right,forward",
        help="Comma-separated labels to train on (default: left,right,forward).",
    )
    p.add_argument("--valid-pct", type=float, default=0.2, help="Validation split ratio.")
    p.add_argument("--seed", type=int, default=42, help="Random seed for split/reproducibility.")
    p.add_argument("--image-size", type=int, default=224, help="Square resize for model input.")
    p.add_argument("--batch-size", type=int, default=32, help="Batch size.")
    p.add_argument("--epochs", type=int, default=8, help="Fine-tune epochs.")
    p.add_argument("--lr", type=float, default=3e-3, help="Learning rate.")
    p.add_argument(
        "--num-workers",
        type=int,
        default=0,
        help="Dataloader workers (default: 0 for Windows pickling reliability).",
    )
    p.add_argument(
        "--arch",
        default="resnet18",
        choices=("resnet18", "resnet34"),
        help="Backbone architecture (default: resnet18).",
    )
    p.add_argument(
        "--output",
        default="models/turn_classifier.pkl",
        help="Path to exported fastai learner (default: models/turn_classifier.pkl).",
    )
    p.add_argument(
        "--show-batch",
        action="store_true",
        help="Show one training batch before training.",
    )
    return p.parse_args()


def _label_from_filename(p: Path) -> str | None:
    # Expected pattern: frame_00012_left.png -> label = "left"
    stem = p.stem
    parts = stem.split("_")
    if not parts:
        return None
    return parts[-1].lower()


def _collect_images(root: Path, run_glob: str, allowed: set[str]) -> list[Path]:
    items: list[Path] = []
    for run_dir in sorted(root.glob(run_glob)):
        if not run_dir.is_dir():
            continue
        images_dir = run_dir / "images"
        if not images_dir.exists():
            continue
        for p in images_dir.glob("*.png"):
            label = _label_from_filename(p)
            if label in allowed:
                items.append(p)
    return items


def _identity_items(src: object) -> list[Path]:
    return list(src)  # type: ignore[arg-type]


def _print_distribution(items: Iterable[Path]) -> Counter:
    counts: Counter = Counter()
    for p in items:
        label = _label_from_filename(p)
        if label is not None:
            counts[label] += 1
    print("Class distribution:")
    for k in sorted(counts.keys()):
        print(f"  {k:>8}: {counts[k]}")
    return counts


def main() -> int:
    args = _parse_args()

    try:
        from fastai.vision.all import (
            CategoryBlock,
            DataBlock,
            ImageBlock,
            RandomSplitter,
            Resize,
            accuracy,
            aug_transforms,
            error_rate,
            resnet18,
            resnet34,
            vision_learner,
        )
    except ImportError as e:
        print(
            "Missing dependency. Install with:\n"
            "  pip install torch torchvision fastai\n"
            f"Details: {e}"
        )
        return 2

    data_root = Path(args.data_root)
    classes = [c.strip().lower() for c in args.classes.split(",") if c.strip()]
    allowed = set(classes)
    if not classes:
        print("No classes provided (--classes).")
        return 2
    if not data_root.exists():
        print(f"Data root not found: {data_root}")
        return 2

    items = _collect_images(data_root, args.run_glob, allowed)
    if not items:
        print(
            f"No matching images found under {data_root} with run glob "
            f"{args.run_glob!r} and classes {sorted(allowed)}."
        )
        return 2

    print(f"Found {len(items)} training images.")
    counts = _print_distribution(items)
    missing = [c for c in classes if counts.get(c, 0) == 0]
    if missing:
        print(f"Warning: classes with 0 samples: {missing}")

    # Keep labels deterministic and explicit.
    vocab = [c for c in classes if counts.get(c, 0) > 0]
    if len(vocab) < 2:
        print(f"Need at least 2 non-empty classes to train. Got: {vocab}")
        return 2

    dblock = DataBlock(
        blocks=(ImageBlock, CategoryBlock(vocab=vocab)),
        get_items=_identity_items,
        get_y=_label_from_filename,
        splitter=RandomSplitter(valid_pct=args.valid_pct, seed=args.seed),
        item_tfms=Resize(args.image_size),
        batch_tfms=aug_transforms(size=args.image_size),
    )
    dls = dblock.dataloaders(
        source=items,
        bs=args.batch_size,
        num_workers=args.num_workers,
    )

    if args.show_batch:
        dls.show_batch(max_n=8)

    arch = resnet18 if args.arch == "resnet18" else resnet34
    learn = vision_learner(dls, arch, metrics=[accuracy, error_rate])
    learn.fine_tune(args.epochs, base_lr=args.lr)

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    learn.export(out_path)
    print(f"Exported model: {out_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

