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
        # Added to base rotation (BOXSIM_SCREENSHOT_CAMERA_ROTATION), e.g. [0, 90, 0] = yaw +90° in Unreal before lit capture.
        "camera_rotation_add_deg": [0.0, 90.0, 0.0],
        "swap_ortho_width_height": False,
        # When false, the saved PNG is exactly what Unreal renders (no OpenCV transpose/rotate/flip).
        "apply_lit_image_transforms": False,
        "transpose_lit_image": False,
        "lit_rotate_90": "none",
        "lit_flip_horizontal": False,
        "lit_flip_vertical": False,
        "origin_world_adjust": [0.0, 0.0],
        # World units added on each side to capture.bounds in aabb mode (expands ortho frustum).
        "bounds_padding": 200.0,
    },
    "pose": {
        "pose_swap_xy": False,
        "pose_pixel_flip_y": True,
        "pose_yaw_offset_deg": 0.0,
        "map_mirror_x": True,
        "map_mirror_y": True,
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

    bpad = os.environ.get("BOXSIM_BOUNDS_PADDING", "").strip()
    if bpad:
        try:
            cap["bounds_padding"] = float(bpad)
        except ValueError:
            pass

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

    if _env_bool("BOXSIM_APPLY_LIT_IMAGE_TRANSFORMS"):
        cap["apply_lit_image_transforms"] = True

    cra = os.environ.get("BOXSIM_CAMERA_ROTATION_ADD_DEG", "").strip()
    if cra:
        parts = [p.strip() for p in cra.split(",")]
        if len(parts) >= 3:
            try:
                cap["camera_rotation_add_deg"] = [float(parts[0]), float(parts[1]), float(parts[2])]
            except ValueError:
                pass

    lr = os.environ.get("BOXSIM_LIT_ROTATE_90", "").strip().lower()
    if lr in ("cw", "clockwise", "ccw", "counterclockwise", "anticlockwise", "none", "0", "false"):
        if lr in ("none", "0", "false"):
            cap["lit_rotate_90"] = "none"
        elif lr in ("ccw", "counterclockwise", "anticlockwise"):
            cap["lit_rotate_90"] = "ccw"
        else:
            cap["lit_rotate_90"] = "cw"
    if _env_bool("BOXSIM_LIT_FLIP_HORIZONTAL"):
        cap["lit_flip_horizontal"] = True
    if _env_bool("BOXSIM_LIT_FLIP_VERTICAL"):
        cap["lit_flip_vertical"] = True

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


def capture_bounds_padding(cfg: dict[str, Any]) -> float:
    """Extra world units to expand aabb bounds on each side (-X,+X,-Y,+Y) for ortho capture."""
    cap = cfg.get("capture") or {}
    raw = cap.get("bounds_padding", 200.0)
    try:
        p = float(raw)
    except (TypeError, ValueError):
        return 200.0
    return max(0.0, p)


def resolve_capture_bounds(cfg: dict[str, Any]) -> tuple[float, float, float, float] | None:
    """UE AABB xmin,xmax,ymin,ymax for screenshot framing (after bounds_padding), or None for pawn-centered XY."""
    cap = cfg.get("capture") or {}
    mode = str(cap.get("mode", "pawn_xy")).lower()
    bounds = cap.get("bounds")
    if mode == "aabb" and bounds is not None and len(bounds) == 4:
        try:
            xmin, xmax, ymin, ymax = (float(bounds[0]), float(bounds[1]), float(bounds[2]), float(bounds[3]))
        except (TypeError, ValueError):
            return None
        if xmax > xmin and ymax > ymin:
            pad = capture_bounds_padding(cfg)
            if pad > 0.0:
                xmin -= pad
                xmax += pad
                ymin -= pad
                ymax += pad
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
        "pose_pixel_flip_y": bool(pose.get("pose_pixel_flip_y", True)),
        "pose_yaw_offset_deg": float(pose.get("pose_yaw_offset_deg", 0.0)),
    }


def map_display_meta_from_config(cfg: dict[str, Any]) -> dict[str, Any]:
    """Pose flags + optional mirrors for map × screen (+UE Y → +screen X uses pose_swap_xy)."""
    pose = cfg.get("pose") or {}
    meta = pose_meta_from_config(cfg)
    meta["map_mirror_x"] = bool(pose.get("map_mirror_x", True))
    meta["map_mirror_y"] = bool(pose.get("map_mirror_y", True))
    return meta


def capture_camera_rotation_add_deg(cfg: dict[str, Any]) -> tuple[float, float, float]:
    """Pitch, yaw, roll (degrees) added to BOXSIM_SCREENSHOT_CAMERA_ROTATION before vset /camera/.../rotation."""
    cap = cfg.get("capture") or {}
    add = cap.get("camera_rotation_add_deg") or [0.0, 0.0, 0.0]
    if len(add) >= 3:
        try:
            return (float(add[0]), float(add[1]), float(add[2]))
        except (TypeError, ValueError):
            pass
    return (0.0, 0.0, 0.0)


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
