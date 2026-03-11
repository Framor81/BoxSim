# ScreenshotMapBuilder: from Unreal. ManualMapBuilder: blank canvas.

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import cv2

from map_utils import scale_from_ortho_width
from .viewer import MapViewer

if TYPE_CHECKING:
    from agent import UnrealAgent

DEFAULT_CAMERA_HEIGHT = 1000
DEFAULT_ORTHO_WIDTH = 4000


class ScreenshotMapBuilder:
    def __init__(
        self,
        *,
        screenshot_path: str | Path = "data/screenshots/topdown.png",
        camera_height: float = DEFAULT_CAMERA_HEIGHT,
        ortho_width: float = DEFAULT_ORTHO_WIDTH,
        viewer_width: int = 1024,
        viewer_height: int = 768,
        save_path_prefix: str = "data/maps/map",
    ) -> None:
        self.screenshot_path = Path(screenshot_path)
        self.camera_height = camera_height
        self.ortho_width = ortho_width
        self.viewer_width = viewer_width
        self.viewer_height = viewer_height
        self.save_path_prefix = save_path_prefix

    def run(self, agent: "UnrealAgent") -> None:
        if not agent.is_connected():
            raise RuntimeError("Agent must be connected")
        saved_path, metadata = self._capture_topdown(agent.client)
        if not saved_path or not Path(saved_path).exists():
            raise RuntimeError("Failed to capture screenshot")
        img = cv2.imread(saved_path)
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

    def _capture_topdown(self, client) -> tuple[str | None, dict]:
        path = self.screenshot_path.resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        name = path.name
        try:
            r = client.request("vset /cameras/spawn")
            cam_id = (r or "").strip()
            if not cam_id or not cam_id.isdigit():
                return (None, {})
            client.request(f"vset /camera/{cam_id}/location 0 0 {self.camera_height}")
            client.request(f"vset /camera/{cam_id}/rotation -90 0 0")
            client.request(f"vset /camera/{cam_id}/projection_type orthographic")
            client.request(f"vset /camera/{cam_id}/ortho_width {self.ortho_width}")
            returned = client.request(f"vget /camera/{cam_id}/lit {name}") or ""
            saved_to = returned.strip()
            img = None
            for p in [path, Path(saved_to), Path(saved_to).name]:
                if p.exists():
                    img = cv2.imread(str(p))
                    if img is not None and p != path:
                        cv2.imwrite(str(path), img)
                    break
            if img is None and saved_to:
                img = cv2.imread(saved_to)
                if img is not None:
                    cv2.imwrite(str(path), img)
            if img is None:
                return (None, {})
            w = img.shape[1]
            return (str(path), {"origin": [0, 0], "scale": scale_from_ortho_width(self.ortho_width, w), "ortho_width": self.ortho_width})
        except Exception:
            return (None, {})


class ManualMapBuilder:
    def __init__(
        self,
        *,
        world_width: float = 1000,
        world_height: float = 1000,
        viewer_width: int = 1024,
        viewer_height: int = 768,
        save_path_prefix: str = "data/maps/manual_map",
    ) -> None:
        self.world_width = world_width
        self.world_height = world_height
        self.viewer_width = viewer_width
        self.viewer_height = viewer_height
        self.save_path_prefix = save_path_prefix

    def run(self, agent: "UnrealAgent | None" = None) -> None:
        scale = self.viewer_width / self.world_width
        metadata = {
            "origin": [0, 0],
            "scale": scale,
            "ortho_width": self.world_width,
            "world_height": self.world_height,
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
