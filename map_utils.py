"""
Map data: world↔pixel coordinates and load/save (map.png + map.json).
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


# --- Coordinates (Unreal world X,Y <-> map pixels) ---

def world_to_pixel(
    world_x: float, world_y: float,
    origin_x: float, origin_y: float,
    scale: float, flip_y: bool = True,
) -> tuple[float, float]:
    px = (world_x - origin_x) * scale
    py = (world_y - origin_y) * scale
    if flip_y:
        py = -py
    return (px, py)


def pixel_to_world(
    pixel_x: float, pixel_y: float,
    origin_x: float, origin_y: float,
    scale: float, flip_y: bool = True,
) -> tuple[float, float]:
    if flip_y:
        pixel_y = -pixel_y
    return (pixel_x / scale + origin_x, pixel_y / scale + origin_y)


def scale_from_ortho_width(ortho_width: float, image_width: int) -> float:
    if ortho_width <= 0:
        return 1.0
    return image_width / ortho_width


# --- Load / save map image + metadata ---

def load_map(path_prefix: str | Path) -> tuple[np.ndarray, dict[str, Any]]:
    path_prefix = Path(path_prefix)
    img_path = path_prefix.with_suffix(".png")
    meta_path = path_prefix.with_suffix(".json")
    if not img_path.exists():
        raise FileNotFoundError(f"Map image not found: {img_path}")
    image = cv2.imread(str(img_path))
    if image is None:
        raise IOError(f"Could not read image: {img_path}")
    metadata: dict[str, Any] = {"origin": [0, 0], "scale": 0.2, "ortho_width": 4000}
    if meta_path.exists():
        with open(meta_path) as f:
            metadata = json.load(f)
    return image, metadata


def save_map(path_prefix: str | Path, image: np.ndarray, metadata: dict[str, Any]) -> None:
    path_prefix = Path(path_prefix)
    path_prefix.parent.mkdir(parents=True, exist_ok=True)
    if image is not None:
        cv2.imwrite(str(path_prefix.with_suffix(".png")), image)
    with open(path_prefix.with_suffix(".json"), "w") as f:
        json.dump(metadata, f, indent=2)
