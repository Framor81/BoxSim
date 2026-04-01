# Load BoxSim capture / pose JSON config; env overrides for CI and one-off runs.

from __future__ import annotations

import json
import os
from copy import deepcopy
from pathlib import Path
from typing import Any

_REPO_ROOT = Path(__file__).resolve().parent.parent
_CONFIG_DIR = _REPO_ROOT / "config"

_DEFAULT_CONFIG: dict[str, Any] = {
    "capture": {
        "mode": "pawn_xy",
        "bounds": None,
        "ortho_width": 2000.0,
        "ortho_height": 2000.0,
        "camera_z_offset": None,
        "swap_ortho_width_height": False,
        "transpose_lit_image": False,
        "origin_world_adjust": [0.0, 0.0],
    },
    "pose": {
        "pose_swap_xy": False,
        "pose_pixel_flip_y": False,
        "pose_yaw_offset_deg": 0.0,
    },
}


def _deep_merge(base: dict, overlay: dict) -> dict:
    out = deepcopy(base)
    for k, v in overlay.items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = deepcopy(v)
    return out


def _config_paths() -> tuple[Path, Path]:
    explicit = os.environ.get("BOXSIM_CONFIG", "").strip()
    if explicit:
        return (Path(os.path.expandvars(explicit)).expanduser(), Path())
    local = _CONFIG_DIR / "boxsim.json"
    example = _CONFIG_DIR / "boxsim.example.json"
    return (local, example)


def load_boxsim_config() -> dict[str, Any]:
    """Merged config: defaults < example/boxsim.json < env overrides (BOXSIM_*)."""
    local, example = _config_paths()
    cfg = deepcopy(_DEFAULT_CONFIG)
    path = local if local.is_file() else example
    if path.is_file():
        with open(path, encoding="utf-8") as f:
            loaded = json.load(f)
        if isinstance(loaded, dict):
            cfg = _deep_merge(cfg, loaded)
    _apply_env_overrides(cfg)
    return cfg


def _env_bool(name: str) -> bool | None:
    v = os.environ.get(name, "").strip().lower()
    if not v:
        return None
    return v in ("1", "true", "yes", "on")


def _apply_env_overrides(cfg: dict[str, Any]) -> None:
    cap = cfg.setdefault("capture", {})
    pose = cfg.setdefault("pose", {})
    raw_bounds = os.environ.get("BOXSIM_CAPTURE_WORLD_BOUNDS", "").strip()
    if raw_bounds:
        parts = [p.strip() for p in raw_bounds.split(",")]
        if len(parts) == 4:
            try:
                cap["bounds"] = [float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])]
                cap["mode"] = "aabb"
            except ValueError:
                pass

    if _env_bool("BOXSIM_CAPTURE_USE_PAWN_XY") and not raw_bounds:
        cap["mode"] = "pawn_xy"

    for key, envname in (
        ("ortho_width", "BOXSIM_ORTHO_WIDTH"),
        ("ortho_height", "BOXSIM_ORTHO_HEIGHT"),
    ):
        raw = os.environ.get(envname, "").strip()
        if raw:
            try:
                cap[key] = float(raw)
            except ValueError:
                pass

    z = os.environ.get("BOXSIM_CAMERA_Z_OFFSET", "").strip()
    if z:
        try:
            cap["camera_z_offset"] = float(z)
        except ValueError:
            pass

    for key, envname in (
        ("pose_swap_xy", "BOXSIM_POSE_SWAP_XY"),
        ("pose_pixel_flip_y", "BOXSIM_POSE_PIXEL_FLIP_Y"),
    ):
        b = _env_bool(envname)
        if b is not None:
            pose[key] = b

    yaw = os.environ.get("BOXSIM_POSE_YAW_OFFSET_DEG", "").strip()
    if yaw:
        try:
            pose["pose_yaw_offset_deg"] = float(yaw)
        except ValueError:
            pass

    if _env_bool("BOXSIM_TRANSPOSE_LIT_IMAGE"):
        cap["transpose_lit_image"] = True
    if _env_bool("BOXSIM_SWAP_ORTHO_WIDTH_HEIGHT"):
        cap["swap_ortho_width_height"] = True

    if _env_bool("BOXSIM_MAP_MIRROR_X"):
        pose["map_mirror_x"] = True
    if _env_bool("BOXSIM_MAP_MIRROR_Y"):
        pose["map_mirror_y"] = True

    oadj = os.environ.get("BOXSIM_ORIGIN_WORLD_ADJUST", "").strip()
    if oadj:
        parts = [p.strip() for p in oadj.split(",")]
        if len(parts) == 2:
            try:
                cap["origin_world_adjust"] = [float(parts[0]), float(parts[1])]
            except ValueError:
                pass


def resolve_capture_bounds(cfg: dict[str, Any]) -> tuple[float, float, float, float] | None:
    """UE AABB xmin,xmax,ymin,ymax for screenshot framing, or None for pawn-centered XY."""
    cap = cfg.get("capture") or {}
    mode = str(cap.get("mode", "pawn_xy")).lower()
    bounds = cap.get("bounds")
    if mode == "aabb" and bounds is not None and len(bounds) == 4:
        try:
            xmin, xmax, ymin, ymax = (float(bounds[0]), float(bounds[1]), float(bounds[2]), float(bounds[3]))
        except (TypeError, ValueError):
            return None
        if xmax > xmin and ymax > ymin:
            return (xmin, xmax, ymin, ymax)
    return None


def capture_ortho_from_config(cfg: dict[str, Any]) -> tuple[float, float]:
    cap = cfg.get("capture") or {}
    w = float(cap.get("ortho_width", _DEFAULT_CONFIG["capture"]["ortho_width"]))
    h = float(cap.get("ortho_height", _DEFAULT_CONFIG["capture"]["ortho_height"]))
    return (w, h)


def capture_camera_z_from_config(cfg: dict[str, Any], default_z: float) -> float:
    cap = cfg.get("capture") or {}
    raw = cap.get("camera_z_offset")
    if raw is None:
        return default_z
    try:
        return float(raw)
    except (TypeError, ValueError):
        return default_z


def pose_meta_from_config(cfg: dict[str, Any]) -> dict[str, Any]:
    pose = cfg.get("pose") or {}
    return {
        "pose_swap_xy": bool(pose.get("pose_swap_xy", False)),
        "pose_pixel_flip_y": bool(pose.get("pose_pixel_flip_y", False)),
        "pose_yaw_offset_deg": float(pose.get("pose_yaw_offset_deg", 0.0)),
    }


def map_display_meta_from_config(cfg: dict[str, Any]) -> dict[str, Any]:
    """Pose flags + optional mirrors for map × screen (+UE Y → +screen X uses pose_swap_xy)."""
    pose = cfg.get("pose") or {}
    meta = pose_meta_from_config(cfg)
    meta["map_mirror_x"] = bool(pose.get("map_mirror_x", False))
    meta["map_mirror_y"] = bool(pose.get("map_mirror_y", False))
    return meta


def apply_origin_world_adjust(cfg: dict[str, Any], cx: float, cy: float) -> tuple[float, float]:
    cap = cfg.get("capture") or {}
    raw = cap.get("origin_world_adjust") or [0.0, 0.0]
    if len(raw) != 2:
        return (cx, cy)
    try:
        dx, dy = float(raw[0]), float(raw[1])
    except (TypeError, ValueError):
        return (cx, cy)
    return (cx + dx, cy + dy)
