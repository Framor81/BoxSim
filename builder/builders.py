# ScreenshotMapBuilder: from Unreal. ManualMapBuilder: blank canvas.

from __future__ import annotations

import os
import re
from pathlib import Path
from typing import TYPE_CHECKING

import cv2
import numpy as np

from map_utils import scales_from_ortho
from .capture_config import (
    capture_camera_z_from_config,
    capture_ortho_from_config,
    load_boxsim_config,
    pose_meta_from_config,
    resolve_capture_bounds,
)
from .viewer import MapViewer

if TYPE_CHECKING:
    from agent import UnrealAgent

# Fallback Z offset above pawn when config has no camera_z_offset (ortho framing is ortho_width/height, not Z).
DEFAULT_CAMERA_HEIGHT = 500.0
# Fallback ortho spans only if config file is missing (see config/boxsim.example.json).
DEFAULT_ORTHO_WIDTH = 2000.0
DEFAULT_ORTHO_HEIGHT = 2000.0

# Capture framing and pose defaults: config/boxsim.json or config/boxsim.example.json; override with BOXSIM_* env vars.
# Image center world XY is stored as metadata "origin"; scale_x/y = native_size / ortho span per axis.

# Lit captures come from your scene sensor (e.g. FusionCamSensor), not camera 0 (pawn/view target).
# Use vget /cameras → pick a non-zero id, or set BOXSIM_UNREALCV_CAMERA_ID=1 (etc.).
# Custom UnrealCV: only location + rotation work; /pose is not implemented.
SCREENSHOT_CAMERA_ROTATION = "-90 0 0"


def _env_bool(name: str, default: bool) -> bool:
    v = os.environ.get(name, "").strip().lower()
    if not v:
        return default
    return v in ("1", "true", "yes", "on")


def _screenshot_rotation_string() -> str:
    return os.environ.get("BOXSIM_SCREENSHOT_CAMERA_ROTATION", SCREENSHOT_CAMERA_ROTATION).strip()


def _resolve_capture_camera_position(
    agent: "UnrealAgent | None",
    default_z: float,
    debug: dict,
) -> tuple[float, float, float]:
    """World position for the *virtual* capture camera only — the pawn is never moved.
    Default: above the pawn (same XY, Z = pawn Z + offset) so the view does not jump to world origin."""
    center = _env_bool("BOXSIM_CAMERA_CENTER_ON_PAWN", True)
    try:
        z_off = float(os.environ.get("BOXSIM_CAMERA_Z_OFFSET", str(default_z)).strip())
    except ValueError:
        z_off = float(default_z)
    if center and agent is not None:
        pose = agent.get_pawn_pose()
        debug["pawn_pose_for_camera"] = repr(pose)
        if pose is not None:
            return (pose.x, pose.y, pose.z + z_off)
    return (0.0, 0.0, float(default_z))


def _debug_screenshot() -> bool:
    return os.environ.get("BOXSIM_DEBUG_SCREENSHOT", "").strip() in ("1", "true", "yes")


def _parse_spawn_camera_id(response: str | None) -> str | None:
    """UnrealCV may return '0', '1', or text with trailing junk — take first line, first integer if needed."""
    if not response or not str(response).strip():
        return None
    line = str(response).strip().splitlines()[0].strip()
    if line.isdigit():
        return line
    m = re.search(r"-?\d+", line)
    return m.group(0) if m else None


def _camera_ids_from_vget_cameras(response: str | None) -> list[int]:
    """Parse integers from `vget /cameras` (format varies by build)."""
    if not response or not str(response).strip():
        return []
    ids: list[int] = []
    for m in re.findall(r"-?\d+", str(response)):
        try:
            ids.append(int(m))
        except ValueError:
            pass
    return sorted(set(ids))


def _resolve_capture_camera_id(client, debug: dict) -> str | None:
    """Lit capture uses FusionCamSensor (id 1+); camera 0 is the pawn, not the render camera."""
    explicit = os.environ.get("BOXSIM_UNREALCV_CAMERA_ID", "").strip()
    if explicit.isdigit():
        debug["camera_id_source"] = "BOXSIM_UNREALCV_CAMERA_ID"
        return explicit

    raw = client.request("vget /cameras")
    debug["vget_cameras_response"] = repr(raw)
    ids = _camera_ids_from_vget_cameras(raw)
    debug["camera_ids_parsed"] = ids

    non_zero = [i for i in ids if i != 0]
    if non_zero:
        chosen = min(non_zero)
        debug["camera_id_source"] = f"vget /cameras (smallest non-zero id → {chosen})"
        return str(chosen)

    if _env_bool("BOXSIM_UNREALCV_SPAWN_FALLBACK", True):
        r = client.request("vset /cameras/spawn")
        debug["spawn_fallback_response"] = repr(r)
        cid = _parse_spawn_camera_id(r)
        if cid and cid != "0":
            debug["camera_id_source"] = "vset /cameras/spawn (non-zero)"
            return cid
        debug["camera_id_source"] = "spawn failed or returned 0"

    return None


def _normalize_path_string(s: str) -> str:
    s = s.strip().strip('"').strip("'")
    return os.path.expandvars(s)


def _imread(path: Path) -> np.ndarray | None:
    """OpenCV imread; on Windows, np.fromfile + imdecode handles some paths imread misses."""
    p = str(path)
    img = cv2.imread(p)
    if img is not None:
        return img
    try:
        if path.is_file():
            data = np.fromfile(str(path.resolve()), dtype=np.uint8)
            return cv2.imdecode(data, cv2.IMREAD_COLOR)
    except OSError:
        pass
    return None


def _candidate_image_paths(dest: Path, vget_response: str) -> list[Path]:
    """Where UnrealCV may write the lit capture (absolute path, cwd-relative, or next to dest)."""
    raw = _normalize_path_string(vget_response)
    out: list[Path] = []
    seen: set[str] = set()

    def add(p: Path) -> None:
        k = str(p.resolve()) if p.is_absolute() else str(p)
        if k not in seen:
            seen.add(k)
            out.append(p)

    add(dest)
    if raw:
        add(Path(raw))
        add(Path(raw).expanduser())
        if not Path(raw).is_absolute():
            add(Path.cwd() / raw)
            add(dest.parent / raw)
    name = dest.name
    add(Path(name))
    add(Path.cwd() / name)
    add(dest.parent / name)
    return out


def _lit_filename_argument(dest: Path) -> list[str]:
    """Try absolute path variants — some UnrealCV builds only accept basename; others need POSIX path."""
    r = dest.resolve()
    variants = [str(r), r.as_posix()]
    if os.name == "nt":
        variants.append(str(r).replace("\\", "/"))
        if " " in str(r):
            variants.append(f'"{r}"')
    out: list[str] = []
    seen: set[str] = set()
    for v in variants:
        if v and v not in seen:
            seen.add(v)
            out.append(v)
    # Basename last: only if absolute paths fail (saves to engine CWD — often not the Python CWD).
    out.append(r.name)
    return out


def _verify_camera_responds(client, cam_id: str, debug: dict) -> str | None:
    """Confirm FusionCam / sensor id responds to vget location (camera 0 may respond but is wrong for lit)."""
    loc = client.request(f"vget /camera/{cam_id}/location")
    rot = client.request(f"vget /camera/{cam_id}/rotation")
    debug["camera_location"] = repr(loc)
    debug["camera_rotation"] = repr(rot)
    if not loc or not str(loc).strip():
        return (
            f"Camera id {cam_id!r} not visible to UnrealCV: "
            f"vget /camera/{cam_id}/location returned empty (wrong id or unsupported on this plugin)"
        )
    parts = str(loc).strip().split()
    if len(parts) < 3:
        return f"vget /camera/{cam_id}/location unexpected: {loc!r}"
    try:
        float(parts[0])
        float(parts[1])
        float(parts[2])
    except ValueError:
        return f"vget /camera/{cam_id}/location not numeric (camera missing?): {loc!r}"
    return None


class ScreenshotMapBuilder:
    def __init__(
        self,
        *,
        screenshot_path: str | Path = "data/screenshots/topdown.png",
        camera_height: float | None = None,
        ortho_width: float | None = None,
        ortho_height: float | None = None,
        viewer_width: int = 1024,
        viewer_height: int = 768,
        save_path_prefix: str = "data/maps/map",
    ) -> None:
        self.screenshot_path = Path(screenshot_path)
        self._cfg = load_boxsim_config()
        cz = capture_camera_z_from_config(self._cfg, DEFAULT_CAMERA_HEIGHT)
        self.camera_height = float(camera_height) if camera_height is not None else cz
        cw, ch = capture_ortho_from_config(self._cfg)
        self.ortho_width = float(ortho_width) if ortho_width is not None else cw
        self.ortho_height = float(ortho_height) if ortho_height is not None else ch
        self.viewer_width = viewer_width
        self.viewer_height = viewer_height
        self.save_path_prefix = save_path_prefix

    def run(self, agent: "UnrealAgent") -> None:
        if not agent.is_connected():
            raise RuntimeError("Agent must be connected")
        saved_path, metadata = self._capture_topdown(agent.client, agent)
        if not saved_path or not Path(saved_path).exists():
            hint = metadata.get("_capture_hint", "")
            raise RuntimeError("Failed to capture screenshot" + (f". {hint}" if hint else ""))
        img = _imread(Path(saved_path))
        if img is None:
            raise RuntimeError(f"Cannot load {saved_path}")
        def pose_getter():
            p = agent.get_pawn_pose()
            return (p.x, p.y, p.yaw) if p else None
        MapViewer(
            self.viewer_width, self.viewer_height,
            metadata, pose_getter,
            background_image=img,
        ).run(self.save_path_prefix)

    def _capture_topdown(self, client, agent: "UnrealAgent | None") -> tuple[str | None, dict]:
        path = self.screenshot_path.resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        debug: dict = {}

        def fail(reason: str) -> tuple[str | None, dict]:
            hint = (
                f"{reason}. Set BOXSIM_DEBUG_SCREENSHOT=1 and retry to print UnrealCV responses. "
                f"Check vget returned a path and that the file exists under Saved/ or project root."
            )
            if _debug_screenshot():
                print("[BOXSIM_DEBUG_SCREENSHOT]", debug)
            return (None, {"_capture_hint": hint, **debug})

        try:
            cam_id = _resolve_capture_camera_id(client, debug)
            if not cam_id:
                return fail(
                    "No capture camera id: vget /cameras had no non-zero id (camera 0 is pawn, not FusionCam lit). "
                    "Set BOXSIM_UNREALCV_CAMERA_ID to your FusionCamSensor id (often 1), or place sensors in the level."
                )
            if cam_id == "0":
                return fail(
                    "Refusing camera id 0 for lit capture (pawn / view target, not FusionCamSensor). "
                    "Set BOXSIM_UNREALCV_CAMERA_ID to a non-zero id from vget /cameras."
                )

            def vset(label: str, cmd: str) -> None:
                resp = client.request(cmd)
                debug[f"vset_{label}"] = repr(resp)

            bounds = resolve_capture_bounds(self._cfg)
            if bounds is not None:
                xmin, xmax, ymin, ymax = bounds
                cx = (xmin + xmax) / 2.0
                cy = (ymin + ymax) / 2.0
                capture_ortho_w = xmax - xmin
                capture_ortho_h = ymax - ymin
                _, _, cz = _resolve_capture_camera_position(agent, float(self.camera_height), debug)
                debug["capture_world_bounds"] = bounds
                if os.environ.get("BOXSIM_CAPTURE_WORLD_BOUNDS", "").strip():
                    debug["capture_xy_source"] = "BOXSIM_CAPTURE_WORLD_BOUNDS env"
                else:
                    debug["capture_xy_source"] = "config capture.mode=aabb"
            else:
                cx, cy, cz = _resolve_capture_camera_position(agent, float(self.camera_height), debug)
                capture_ortho_w = float(self.ortho_width)
                capture_ortho_h = float(self.ortho_height)
                debug["capture_xy_source"] = "pawn_xy / config ortho"

            rot_str = _screenshot_rotation_string()
            debug["capture_camera_id"] = cam_id
            debug["camera_world_xyz"] = (cx, cy, cz)
            debug["camera_rotation_cmd"] = rot_str
            # Ortho + width, then location + rotation only (no /pose — not implemented on custom UnrealCV).
            vset("projection_type", f"vset /camera/{cam_id}/projection_type orthographic")
            vset("ortho_width", f"vset /camera/{cam_id}/ortho_width {capture_ortho_w}")
            vset("location", f"vset /camera/{cam_id}/location {cx} {cy} {cz}")
            vset("rotation", f"vset /camera/{cam_id}/rotation {rot_str}")
            oh_raw = os.environ.get("BOXSIM_ORTHO_HEIGHT", "").strip()
            try:
                oh_used = float(oh_raw) if oh_raw else float(capture_ortho_h)
            except ValueError:
                oh_used = float(capture_ortho_h)
            vset("ortho_height", f"vset /camera/{cam_id}/ortho_height {oh_used}")

            cam_err = _verify_camera_responds(client, cam_id, debug)
            if cam_err:
                return fail(cam_err)

            img = None
            used: Path | None = None
            saved_to = ""
            # Prefer full path so the engine writes where we can read (basename-only saves to an arbitrary CWD).
            for i, lit_arg in enumerate(_lit_filename_argument(path)):
                returned = client.request(f"vget /camera/{cam_id}/lit {lit_arg}") or ""
                debug[f"vget_lit_attempt_{i}"] = repr(returned)
                saved_to = returned.strip()
                if not saved_to:
                    continue
                for cand in _candidate_image_paths(path, saved_to):
                    if cand.is_file():
                        img = _imread(cand)
                        if img is not None:
                            used = cand
                            break
                if img is not None:
                    break
                if path.is_file():
                    img = _imread(path)
                    if img is not None:
                        used = path
                        break

            debug["vget_response_final"] = repr(saved_to)
            debug["tried_exists"] = [
                str(c) for c in _candidate_image_paths(path, saved_to) if c.is_file()
            ] if saved_to else []

            if img is None:
                if not saved_to:
                    return fail(
                        f"vget /camera/{cam_id}/lit returned empty for path attempts "
                        f"{_lit_filename_argument(path)!r}"
                    )
                return fail(
                    f"Could not read image; vget returned {saved_to!r}. "
                    f"Files found among candidates: {debug.get('tried_exists', [])}"
                )
            if used != path:
                cv2.imwrite(str(path), img)
            native_h, native_w = img.shape[0], img.shape[1]
            sx, sy = scales_from_ortho(capture_ortho_w, float(oh_used), native_w, native_h)
            meta = {
                "origin": [float(cx), float(cy)],
                "scale": float(sx),
                "scale_x": float(sx),
                "scale_y": float(sy),
                "ortho_width": float(capture_ortho_w),
                "ortho_height": float(oh_used),
                "capture_native_width": int(native_w),
                "capture_native_height": int(native_h),
                **pose_meta_from_config(self._cfg),
            }
            if bounds is not None:
                meta["capture_world_bounds"] = [bounds[0], bounds[1], bounds[2], bounds[3]]
                meta["world_bounds"] = meta["capture_world_bounds"]
            if _debug_screenshot():
                print("[BOXSIM_DEBUG_SCREENSHOT] ok, loaded from", used, "wrote", path)
            return (str(path), meta)
        except Exception as e:
            debug["exception"] = repr(e)
            return fail(f"Exception during capture: {e}")


class ManualMapBuilder:
    def __init__(
        self,
        *,
        world_width: float | None = None,
        world_height: float | None = None,
        viewer_width: int = 1024,
        viewer_height: int = 768,
        save_path_prefix: str = "data/maps/manual_map",
    ) -> None:
        self._cfg = load_boxsim_config()
        cw, ch = capture_ortho_from_config(self._cfg)
        self.world_width = float(world_width) if world_width is not None else cw
        self.world_height = float(world_height) if world_height is not None else ch
        self.viewer_width = viewer_width
        self.viewer_height = viewer_height
        self.save_path_prefix = save_path_prefix

    def run(self, agent: "UnrealAgent | None" = None) -> None:
        scale_x = self.viewer_width / self.world_width
        scale_y = self.viewer_height / self.world_height
        half_w, half_h = self.world_width / 2.0, self.world_height / 2.0
        metadata = {
            "origin": [0.0, 0.0],
            "scale": scale_x,
            "scale_x": scale_x,
            "scale_y": scale_y,
            "ortho_width": self.world_width,
            "ortho_height": self.world_height,
            "world_height": self.world_height,
            "world_bounds": [-half_w, half_w, -half_h, half_h],
            **pose_meta_from_config(self._cfg),
        }
        def pose_getter():
            if agent is None or not agent.is_connected():
                return None
            p = agent.get_pawn_pose()
            return (p.x, p.y, p.yaw) if p else None
        MapViewer(
            self.viewer_width, self.viewer_height,
            metadata, pose_getter,
            background_image=None,
            world_width=self.world_width,
            world_height=self.world_height,
        ).run(self.save_path_prefix)
