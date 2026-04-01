# Map viewer. 1/2/3=shape, A=obstacle S=drivable D=cut, W=goal E=path, S=Save.
#
# Data model: terrain is lists of TerrainPoly (exterior + optional holes). Cuts subtract from
# terrain (Shapely difference); drawing (brush/polygon) adds. Holes can contain other polygons
# (e.g. drivable "islands" inside a cut); the hole-fill mask excludes those so they stay visible.

from __future__ import annotations

import math
import os
import time
from typing import Callable

import pygame
import numpy as np
from shapely.geometry import Polygon, Point
from shapely import make_valid

import map_utils

# Terrain polygon: (exterior, interiors). exterior = list of (x,y), interiors = list of list of (x,y) (holes).
TerrainPoly = tuple[list[tuple[float, float]], list[list[tuple[float, float]]]]
ErasePoly = TerrainPoly

TOOL_POLYGON, TOOL_BRUSH, TOOL_RECT = "polygon", "brush", "rect"
TOOL_GOAL, TOOL_PATH = "goal", "path"
TERRAIN_OBSTACLE, TERRAIN_DRIVABLE, TERRAIN_CUT = "obstacle", "drivable", "cut"
COLOR_BG = (40, 44, 52)
COLOR_GRID = (60, 64, 72)
COLOR_OBSTACLE = (200, 60, 60)
COLOR_DRIVABLE = (240, 240, 240)
COLOR_OBSTACLE_PREVIEW = (220, 100, 100)
COLOR_DRIVABLE_PREVIEW = (200, 200, 200)
COLOR_CUT_PREVIEW = (100, 108, 120)
COLOR_GOAL = (80, 220, 80)
COLOR_PATH = (255, 220, 80)
COLOR_ROBOT = (100, 180, 100)
COLOR_ROBOT_OUTLINE = (60, 120, 60)
COLOR_AXIS = (100, 100, 120)
GRID_OVERLAY_GRID_ALPHA = 110
GRID_OVERLAY_AXIS_ALPHA = 230
GRID_OVERLAY_LABEL = (0, 0, 0)
GRID_OVERLAY_LABEL_PAD_X = 18
GRID_OVERLAY_LABEL_PAD_Y = 18
BRUSH_RADIUS = 8
GRID_SIZE = 50
POSE_POLL_INTERVAL = 0.2
CLOSE_POINT_RADIUS = 12
COLOR_TOOL_ACTIVE = (120, 255, 120)
GOAL_RADIUS = 10
COORD_TICK_INTERVAL = 200
GRID_LABEL_PAD_X = 6
GRID_LABEL_PAD_Y = 6
# When map.json has no world_bounds / capture_world_bounds / ortho, grid ticks use this symmetric range.
GRID_EXTENT_FALLBACK = (-1000.0, 1000.0, -1000.0, 1000.0)
SIDEBAR_WIDTH = 92


def _robot_display_yaw_rad(yaw_deg: float, offset_deg: float, pose_swap_xy: bool) -> float:
    """UE yaw (Z, degrees) → radians for triangle nose. swap_xy: heading in UE XY matches map after (y,x) position swap."""
    psi = math.radians(yaw_deg + offset_deg)
    if pose_swap_xy:
        return math.pi / 2 - psi
    return psi + math.pi


def _pose_pixel_flip_y_from_config(metadata: dict, has_background_image: bool) -> bool:
    """Robot overlay Y: True = negate world Y in pixel math (grid). False = match Unreal ortho screenshots (Y ~ image down)."""
    v = os.environ.get("BOXSIM_POSE_PIXEL_FLIP_Y", "").strip().lower()
    if v in ("1", "true", "yes"):
        return True
    if v in ("0", "false", "no"):
        return False
    if "pose_pixel_flip_y" in metadata:
        return bool(metadata["pose_pixel_flip_y"])
    return not has_background_image


def _pose_swap_xy_from_config(metadata: dict, has_background_image: bool) -> bool:
    """UE XY vs map: ortho shot is often rotated 90° — pawn +X in world should track map vertical (+map y / screen), not map horizontal."""
    v = os.environ.get("BOXSIM_POSE_SWAP_XY", "").strip().lower()
    if v in ("1", "true", "yes"):
        return True
    if v in ("0", "false", "no"):
        return False
    if "pose_swap_xy" in metadata:
        return bool(metadata["pose_swap_xy"])
    return False


def _pose_yaw_offset_deg_from_config(metadata: dict) -> float:
    raw = os.environ.get("BOXSIM_POSE_YAW_OFFSET_DEG", "").strip()
    if raw:
        try:
            return float(raw)
        except ValueError:
            pass
    v = metadata.get("pose_yaw_offset_deg")
    if v is not None:
        try:
            return float(v)
        except (TypeError, ValueError):
            pass
    return 0.0


def _shapely_to_erase_polys(geom) -> list[ErasePoly]:
    """Convert Shapely geometry to list of (exterior, interiors) for terrain storage.
    Handles: Polygon (one item with exterior + interiors), MultiPolygon (one item per poly),
    GeometryCollection (recurse). Edge cases: None/empty -> []; degenerate polygon (<3 pts) -> skip."""
    if geom is None or geom.is_empty:
        return []
    out: list[ErasePoly] = []
    if geom.geom_type == "Polygon":
        ext = [(float(x), float(y)) for x, y in geom.exterior.coords[:-1]]
        if len(ext) < 3:
            return []
        ints = [[(float(x), float(y)) for x, y in interior.coords[:-1]] for interior in geom.interiors]
        out.append((ext, ints))
    elif geom.geom_type == "MultiPolygon":
        for p in geom.geoms:
            out.extend(_shapely_to_erase_polys(p))
    elif geom.geom_type == "GeometryCollection":
        for g in geom.geoms:
            out.extend(_shapely_to_erase_polys(g))
    return out


def _terrain_poly_to_geom(points: list[tuple[float, float]]):
    """Build a valid Shapely Polygon from a list of (x,y) for use in difference/union.
    Edge cases: <3 points -> None; invalid polygon -> make_valid; still empty after fix -> None."""
    if len(points) < 3:
        return None
    try:
        p = Polygon(points)
        if p.is_empty:
            return None
        if not p.is_valid:
            p = make_valid(p)
        if p.is_empty:
            return None
        return p
    except Exception:
        return None


def _shapely_to_simple_polys(geom) -> list[list[tuple[float, float]]]:
    """Convert Shapely geometry to list of simple polygon coords (exterior only, no holes).
    Used when we only need outlines. Handles Polygon/MultiPolygon/GeometryCollection; skips <3 pts."""
    if geom is None or geom.is_empty:
        return []
    out: list[list[tuple[float, float]]] = []
    if geom.geom_type == "Polygon":
        ext = [(float(x), float(y)) for x, y in geom.exterior.coords[:-1]]
        if len(ext) >= 3:
            out.append(ext)
    elif geom.geom_type == "MultiPolygon":
        for p in geom.geoms:
            out.extend(_shapely_to_simple_polys(p))
    elif geom.geom_type == "GeometryCollection":
        for g in geom.geoms:
            out.extend(_shapely_to_simple_polys(g))
    return out


def _draw_robot_triangle(surface: pygame.Surface, center: tuple[float, float],
                         yaw_rad: float, size: float = 12) -> None:
    """Draw a small triangle for robot pose (nose along yaw_rad) in screen coords."""
    nose = (center[0] + size * math.cos(yaw_rad), center[1] - size * math.sin(yaw_rad))
    back = yaw_rad + math.pi
    half = size * 0.6
    bl = (center[0] + half * math.cos(back + 0.4), center[1] - half * math.sin(back + 0.4))
    br = (center[0] + half * math.cos(back - 0.4), center[1] - half * math.sin(back - 0.4))
    pts = [nose, bl, br]
    pygame.draw.polygon(surface, COLOR_ROBOT, pts)
    pygame.draw.polygon(surface, COLOR_ROBOT_OUTLINE, pts, 2)


class MapViewer:
    """Pygame map builder: draw obstacles/drivable/cuts, goals, path; save to image + JSON."""

    def __init__(
        self,
        width: int, height: int,
        metadata: dict,
        pose_getter: Callable[[], tuple[float, float, float] | None],
        *,
        background_image: np.ndarray | None = None,
        world_width: float | None = None,
        world_height: float | None = None,
    ) -> None:
        """Set up window, load obstacles/drivable from metadata, apply saved erase_polygons as cuts."""
        self.width, self.height = width, height
        self.metadata = dict(metadata)
        self.pose_getter = pose_getter
        self.world_width = world_width or self.metadata.get("ortho_width", 4000)
        self.world_height = world_height or self.metadata.get("world_height") or self.world_width
        self.origin = tuple(self.metadata.get("origin", [0, 0]))
        _sx = float(self.metadata.get("scale_x", self.metadata.get("scale", 0.2)))
        _sy = float(self.metadata.get("scale_y", _sx))
        self.scale_x = _sx
        self.scale_y = _sy
        self.scale = _sx
        self._map_mirror_x = bool(self.metadata.get("map_mirror_x", False))
        self._map_mirror_y = bool(self.metadata.get("map_mirror_y", False))
        self.obstacles = self._load_terrain_polys(self.metadata.get("obstacles", []))
        self.drivable = self._load_terrain_polys(self.metadata.get("drivable", []))
        loaded_erase = self._load_erase_polygons(self.metadata.get("erase_polygons", []))
        self.erase_polygons = []
        for (ext, ints) in loaded_erase:
            try:
                cut_geom = Polygon(ext, ints) if ints else Polygon(ext)
                if not cut_geom.is_valid:
                    cut_geom = make_valid(cut_geom)
                if cut_geom is not None and not cut_geom.is_empty:
                    self._subtract_cut_from_terrain(cut_geom)
            except Exception:
                pass
        self.current_polygon: list[tuple[float, float]] = []
        self.rect_start = self.rect_end = None
        self.brush_surface = None
        self.drivable_brush_surface = None
        self.erase_brush_surface = None
        self.tool = TOOL_POLYGON
        self.terrain = TERRAIN_OBSTACLE
        self.goals = [(float(p[0]), float(p[1])) for p in self.metadata.get("goals", [])]
        self.path = [(float(p[0]), float(p[1])) for p in self.metadata.get("path", [])]
        self.current_path: list[tuple[float, float]] = []
        self.view_offset_x = self.view_offset_y = 0.0
        self._undo_history: list[dict] = []
        self._bg_surface: pygame.Surface | None = None
        self._grid_overlay: pygame.Surface | None = None

        self._capture_native_w = int(self.metadata.get("capture_native_width") or 0)
        self._capture_native_h = int(self.metadata.get("capture_native_height") or 0)
        if background_image is not None:
            self._sync_capture_native_and_scales_from_image(background_image)
            self._reconcile_screenshot_origin()

        self.view_center = (width // 2, height // 2)
        pygame.init()
        self._font = pygame.font.Font(None, 24)
        if background_image is not None:
            if background_image.ndim == 2:
                bg = pygame.surfarray.make_surface(np.stack([background_image] * 3, axis=-1))
            else:
                bg = pygame.surfarray.make_surface(np.transpose(background_image, (1, 0, 2)))
            self.background = pygame.transform.scale(bg, (width, height))
        else:
            self.background = None

        self.screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
        pygame.display.set_caption("Map Builder")
        self.clock = pygame.time.Clock()
        has_bg = self.background is not None
        self._pose_pixel_flip_y = _pose_pixel_flip_y_from_config(self.metadata, has_bg)
        self._pose_swap_xy = _pose_swap_xy_from_config(self.metadata, has_bg)
        self._pose_yaw_offset_deg = _pose_yaw_offset_deg_from_config(self.metadata)
        self._make_brush_surfaces()
        self._build_bg_surface()
        self._build_grid_overlay()
        self.running = True
        self._pose_cache: tuple[float, float, float] | None = None
        self._pose_cache_time = 0.0
        self._save_flash_until = 0.0

    def _sync_capture_native_and_scales_from_image(self, background_image: np.ndarray) -> None:
        """Use real H×W from the lit array; refresh scale_x/y from ortho if JSON size was wrong."""
        if background_image.ndim < 2:
            return
        ah, aw = int(background_image.shape[0]), int(background_image.shape[1])
        if ah <= 0 or aw <= 0:
            return
        if self._capture_native_w != aw or self._capture_native_h != ah:
            self._capture_native_w, self._capture_native_h = aw, ah
        ow = float(self.metadata.get("ortho_width", 0) or 0)
        oh = float(self.metadata.get("ortho_height", 0) or 0)
        if ow > 0 and oh > 0:
            self.scale_x, self.scale_y = map_utils.scales_from_ortho(ow, oh, aw, ah)
            self.scale = self.scale_x

    def _reconcile_screenshot_origin(self) -> None:
        """Fix grid/agent vs bitmap when origin is missing (0,0) or far from capture AABB center."""
        raw = self.metadata.get("capture_world_bounds") or self.metadata.get("world_bounds")
        ctr = self.metadata.get("capture_bounds_center")
        bx: float | None = None
        by: float | None = None
        if ctr and len(ctr) == 2:
            try:
                bx, by = float(ctr[0]), float(ctr[1])
            except (TypeError, ValueError):
                pass
        if bx is None and raw and len(raw) == 4:
            try:
                bx = (float(raw[0]) + float(raw[1])) / 2.0
                by = (float(raw[2]) + float(raw[3])) / 2.0
            except (TypeError, ValueError):
                return
        if bx is None or by is None:
            return
        ox, oy = float(self.origin[0]), float(self.origin[1])
        span = 0.0
        if raw and len(raw) == 4:
            try:
                span = max(float(raw[1]) - float(raw[0]), float(raw[3]) - float(raw[2]))
            except (TypeError, ValueError):
                span = 0.0
        at_zero = abs(ox) < 1e-9 and abs(oy) < 1e-9
        far = span > 0 and math.hypot(ox - bx, oy - by) > 0.25 * span
        if at_zero or far:
            ox, oy = bx, by
        self.origin = (ox, oy)

    def _make_brush_surfaces(self) -> None:
        """Create/resize overlay surfaces for brush and erase; black fill, colorkey black for transparency."""
        for name in ("brush_surface", "drivable_brush_surface", "erase_brush_surface"):
            s = pygame.Surface((self.width, self.height))
            s.set_colorkey((0, 0, 0))
            s.fill((0, 0, 0))
            setattr(self, name, s)

    def _world_to_map_pixel(self, wx: float, wy: float) -> tuple[float, float]:
        """Map-space offsets (native-lit pixels, Y per pose_pixel_flip_y) — same frame as robot overlay."""
        return map_utils.world_to_pixel(
            wx,
            wy,
            self.origin[0],
            self.origin[1],
            self.scale_x,
            flip_y=self._pose_pixel_flip_y,
            scale_y=self.scale_y,
        )

    def _grid_world_extent(self) -> tuple[float, float, float, float]:
        """xmin,xmax,ymin,ymax in world units for axis ticks."""
        raw = self.metadata.get("capture_world_bounds") or self.metadata.get("world_bounds")
        if raw is not None and len(raw) == 4:
            try:
                a, b, c, d = (float(raw[0]), float(raw[1]), float(raw[2]), float(raw[3]))
                if b > a and d > c:
                    return (a, b, c, d)
            except (TypeError, ValueError):
                pass
        ow = float(self.metadata.get("ortho_width", 0.0) or 0.0)
        oh = float(
            self.metadata.get("ortho_height")
            or self.metadata.get("world_height")
            or ow
            or 0.0
        )
        if ow <= 0 or oh <= 0:
            ox, oy = float(self.origin[0]), float(self.origin[1])
            fb = GRID_EXTENT_FALLBACK
            return (ox + fb[0], ox + fb[1], oy + fb[2], oy + fb[3])
        ox, oy = float(self.origin[0]), float(self.origin[1])
        return (ox - ow / 2.0, ox + ow / 2.0, oy - oh / 2.0, oy + oh / 2.0)

    def _draw_world_coord_grid(self, surface: pygame.Surface, *, overlay: bool) -> None:
        """World-space grid; origin + scale match metadata (image center = origin in UE). overlay: semi-transparent."""
        vc = (self.view_center[0] + self.view_offset_x, self.view_center[1] + self.view_offset_y)
        ox, oy = self.origin[0], self.origin[1]
        step = COORD_TICK_INTERVAL
        pad_x = GRID_OVERLAY_LABEL_PAD_X if overlay else GRID_LABEL_PAD_X
        pad_y = GRID_OVERLAY_LABEL_PAD_Y if overlay else GRID_LABEL_PAD_Y
        if overlay:
            c_grid = (*COLOR_GRID, GRID_OVERLAY_GRID_ALPHA)
            c_axis = (*COLOR_AXIS, GRID_OVERLAY_AXIS_ALPHA)
            c_label = GRID_OVERLAY_LABEL
        else:
            c_grid, c_axis, c_label = COLOR_GRID, COLOR_AXIS, COLOR_AXIS
        gx0, gx1, gy0, gy1 = self._grid_world_extent()
        wx = gx0
        while wx <= gx1:
            mx, _ = self._world_to_map_pixel(wx, oy)
            px, _ = self._map_pixel_to_screen(mx, 0.0)
            if -20 <= px < self.width + 20:
                w = 2 if abs(wx) < 1e-9 else 1
                col = c_axis if abs(wx) < 1e-9 else c_grid
                pygame.draw.line(surface, col, (int(px), 0), (int(px), self.height), w)
                t = self._font.render(str(int(wx)) if wx == int(wx) else str(round(wx, 2)), True, c_label)
                surface.blit(t, (int(px) - t.get_width() // 2, int(vc[1]) + pad_y))
            wx += step
        world_y = gy0
        while world_y <= gy1:
            _, my = self._world_to_map_pixel(ox, world_y)
            _, py = self._map_pixel_to_screen(0.0, my)
            if -20 <= py < self.height + 20:
                w = 2 if abs(world_y) < 1e-9 else 1
                col = c_axis if abs(world_y) < 1e-9 else c_grid
                pygame.draw.line(surface, col, (0, int(py)), (self.width, int(py)), w)
                lab = str(int(world_y)) if world_y == int(world_y) else str(round(world_y, 2))
                t = self._font.render(lab, True, c_label)
                surface.blit(t, (int(vc[0]) - t.get_width() - pad_x, int(py) - t.get_height() // 2))
            world_y += step

    def _build_bg_surface(self) -> None:
        """Build grid background (when no image): axes and ticks in map coords; used for hole fill and base."""
        if self.background is not None:
            self._bg_surface = None
            return
        self._bg_surface = pygame.Surface((self.width, self.height))
        self._bg_surface.fill(COLOR_BG)
        self._draw_world_coord_grid(self._bg_surface, overlay=False)

    def _build_grid_overlay(self) -> None:
        """Semi-transparent grid + axis labels on top of screenshot (same world ticks as manual builder)."""
        if self.background is None:
            self._grid_overlay = None
            return
        self._grid_overlay = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        self._grid_overlay.fill((0, 0, 0, 0))
        self._draw_world_coord_grid(self._grid_overlay, overlay=True)

    def _draw_grid(self, surface: pygame.Surface) -> None:
        """Draw base layer: grid background or solid fill if no _bg_surface."""
        if self._bg_surface is not None:
            surface.blit(self._bg_surface, (0, 0))
        else:
            surface.fill(COLOR_BG)

    def _map_window_scale_k(self) -> tuple[float, float]:
        """Map coords are in native-lit space (world * scale); scale to pygame when screenshot is stretched to window."""
        if (
            self.background is not None
            and self._capture_native_w > 0
            and self._capture_native_h > 0
        ):
            return (
                self.width / self._capture_native_w,
                self.height / self._capture_native_h,
            )
        return (1.0, 1.0)

    def _screen_to_map_pixel(self, sx: float, sy: float) -> tuple[float, float]:
        """Screen (pixel) to map coords: origin at view_center + offset, Y flipped."""
        kx, ky = self._map_window_scale_k()
        mx = (sx - self.view_center[0] - self.view_offset_x) / kx
        my = -(sy - self.view_center[1] - self.view_offset_y) / ky
        if self._map_mirror_x:
            mx = -mx
        if self._map_mirror_y:
            my = -my
        return (mx, my)

    def _map_pixel_to_screen(self, mx: float, my: float) -> tuple[float, float]:
        """Map coords to screen (pixel). Inverse of _screen_to_map_pixel."""
        if self._map_mirror_x:
            mx = -mx
        if self._map_mirror_y:
            my = -my
        kx, ky = self._map_window_scale_k()
        return (
            self.view_center[0] + mx * kx + self.view_offset_x,
            self.view_center[1] - my * ky + self.view_offset_y,
        )

    def _load_terrain_polys(self, raw: list) -> list[TerrainPoly]:
        """Load terrain from JSON: legacy list of [[x,y],...] or list of {exterior, interiors}. Skips <3 pts."""
        out: list[TerrainPoly] = []
        for item in raw:
            if isinstance(item, dict):
                ext = [tuple(p) for p in item.get("exterior", [])]
                ints = [[tuple(p) for p in hole] for hole in item.get("interiors", [])]
                if len(ext) >= 3:
                    out.append((ext, ints))
            else:
                poly = [(float(x), float(y)) for x, y in item]
                if len(poly) >= 3:
                    out.append((poly, []))
        return out

    def _load_erase_polygons(self, raw: list) -> list[ErasePoly]:
        """Load erase (cut) polygons from JSON; same format as _load_terrain_polys."""
        return self._load_terrain_polys(raw)

    def _subtract_cut_from_terrain(self, cut_geom) -> None:
        """Cut = eraser: subtract cut_geom from every obstacle and drivable polygon (Shapely difference).
        Cases: cut fully inside polygon -> new hole; cut crossing boundary -> split/reshape; cut outside -> no change.
        Preserves existing holes; can create nested holes (cut inside hole). Invalid/empty geom skipped; on exception keep original poly."""
        if cut_geom is None or cut_geom.is_empty:
            return
        if not cut_geom.is_valid:
            cut_geom = make_valid(cut_geom)
        if cut_geom.is_empty:
            return

        def subtract_from_list(poly_list: list[TerrainPoly]) -> list[TerrainPoly]:
            new_list: list[TerrainPoly] = []
            for (ext, ints) in poly_list:
                if len(ext) < 3:
                    continue
                try:
                    p = Polygon(ext, ints) if ints else Polygon(ext)
                    if p.is_empty:
                        continue
                    if not p.is_valid:
                        p = make_valid(p)
                    if p.is_empty:
                        continue
                    diff = p.difference(cut_geom)
                    new_list.extend(_shapely_to_erase_polys(diff))
                except Exception:
                    new_list.append((ext, ints))
            return new_list

        self.obstacles = subtract_from_list(self.obstacles)
        self.drivable = subtract_from_list(self.drivable)

    def _push_undo_state(self) -> None:
        """Snapshot current obstacles, drivable, erase_polygons, goals, path for undo."""
        self._undo_history.append({
            "obstacles": [(list(ext), [list(h) for h in ints]) for (ext, ints) in self.obstacles],
            "drivable": [(list(ext), [list(h) for h in ints]) for (ext, ints) in self.drivable],
            "erase_polygons": [(list(ext), [list(h) for h in ints]) for (ext, ints) in self.erase_polygons],
            "goals": list(self.goals),
            "path": list(self.path),
        })

    def _undo(self) -> None:
        """Restore last snapshot; no-op if history empty."""
        if not self._undo_history:
            return
        state = self._undo_history.pop()
        self.obstacles = [(list(ext), [list(h) for h in ints]) for (ext, ints) in state["obstacles"]]
        self.drivable = [(list(ext), [list(h) for h in ints]) for (ext, ints) in state["drivable"]]
        self.erase_polygons = [(list(ext), [list(h) for h in ints]) for (ext, ints) in state["erase_polygons"]]
        self.goals = list(state["goals"])
        self.path = list(state["path"])

    def _clear_erase_region_screen(self, screen_pts: list[tuple[float, float]]) -> None:
        """Clear this polygon region on erase_brush_surface (draw black) so new obstacle/drivable is visible."""
        if self.erase_brush_surface is None or len(screen_pts) < 3:
            return
        pts = [(int(p[0]), int(p[1])) for p in screen_pts]
        pygame.draw.polygon(self.erase_brush_surface, (0, 0, 0), pts)

    def _clear_brush_region_screen(self, screen_pts: list[tuple[float, float]]) -> None:
        """Clear this polygon on brush and drivable_brush surfaces so the cut removes painted strokes there."""
        if len(screen_pts) < 3:
            return
        pts = [(int(p[0]), int(p[1])) for p in screen_pts]
        if self.brush_surface is not None:
            pygame.draw.polygon(self.brush_surface, (0, 0, 0), pts)
        if self.drivable_brush_surface is not None:
            pygame.draw.polygon(self.drivable_brush_surface, (0, 0, 0), pts)

    def _commit_polygon(self) -> None:
        """Commit current_polygon: add as obstacle/drivable (and clear erase in that region) or apply as cut.
        Works for polygons fully inside holes (adds as new poly; hole-fill mask keeps it visible). <3 pts no-op."""
        if len(self.current_polygon) < 3:
            return
        self._push_undo_state()
        poly = self.current_polygon.copy()
        if self.terrain == TERRAIN_OBSTACLE:
            self.obstacles.append((poly, []))
            pts = [self._map_pixel_to_screen(p[0], p[1]) for p in poly]
            self._clear_erase_region_screen(pts)
        elif self.terrain == TERRAIN_DRIVABLE:
            self.drivable.append((poly, []))
            pts = [self._map_pixel_to_screen(p[0], p[1]) for p in poly]
            self._clear_erase_region_screen(pts)
        else:
            cut_geom = _terrain_poly_to_geom(poly)
            if cut_geom is not None:
                self._subtract_cut_from_terrain(cut_geom)
            pts = [self._map_pixel_to_screen(p[0], p[1]) for p in poly]
            self._clear_brush_region_screen(pts)

    def _commit_rect(self) -> None:
        """Commit box from rect_start/rect_end: add as obstacle/drivable or apply as cut. Degenerate rect (<1px) discarded."""
        if not self.rect_start or not self.rect_end:
            return
        x0, y0, x1, y1 = *self.rect_start, *self.rect_end
        if abs(x1 - x0) < 1 and abs(y1 - y0) < 1:
            self.rect_start = self.rect_end = None
            return
        self._push_undo_state()
        poly = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        if self.terrain == TERRAIN_OBSTACLE:
            self.obstacles.append((poly, []))
            pts = [self._map_pixel_to_screen(p[0], p[1]) for p in poly]
            self._clear_erase_region_screen(pts)
        elif self.terrain == TERRAIN_DRIVABLE:
            self.drivable.append((poly, []))
            pts = [self._map_pixel_to_screen(p[0], p[1]) for p in poly]
            self._clear_erase_region_screen(pts)
        else:
            cut_geom = _terrain_poly_to_geom(poly)
            if cut_geom is not None:
                self._subtract_cut_from_terrain(cut_geom)
            pts = [self._map_pixel_to_screen(p[0], p[1]) for p in poly]
            self._clear_brush_region_screen(pts)
        self.rect_start = self.rect_end = None

    def _brush_draw(self, pos: tuple[int, int]) -> None:
        """One brush stroke at screen pos: obstacle/drivable = draw circle on brush surface; cut = subtract circle from terrain and clear brush there. Works anywhere (e.g. inside holes)."""
        if self.terrain == TERRAIN_OBSTACLE and self.brush_surface:
            pygame.draw.circle(self.brush_surface, COLOR_OBSTACLE, pos, BRUSH_RADIUS)
            if self.erase_brush_surface is not None:
                pygame.draw.circle(self.erase_brush_surface, (0, 0, 0), pos, BRUSH_RADIUS)
        elif self.terrain == TERRAIN_DRIVABLE and self.drivable_brush_surface:
            pygame.draw.circle(self.drivable_brush_surface, COLOR_DRIVABLE, pos, BRUSH_RADIUS)
            if self.erase_brush_surface is not None:
                pygame.draw.circle(self.erase_brush_surface, (0, 0, 0), pos, BRUSH_RADIUS)
        elif self.terrain == TERRAIN_CUT:
            mx, my = self._screen_to_map_pixel(pos[0], pos[1])
            try:
                circle = Point(mx, my).buffer(BRUSH_RADIUS)
                if not circle.is_empty:
                    self._subtract_cut_from_terrain(circle)
            except Exception:
                pass
            if self.brush_surface is not None:
                pygame.draw.circle(self.brush_surface, (0, 0, 0), pos, BRUSH_RADIUS)
            if self.drivable_brush_surface is not None:
                pygame.draw.circle(self.drivable_brush_surface, (0, 0, 0), pos, BRUSH_RADIUS)

    def _draw_sidebar(self) -> None:
        """Draw tool keys (1/2/3), terrain (A/S/D), goal/path (W/E), save/undo, Esc."""
        default = (200, 200, 200)
        line_h = self._font.get_height() + 4
        gap = 12
        y = 8
        # Tools
        for key, label, active in [
            ("1", "Poly", self.tool == TOOL_POLYGON),
            ("2", "Brush", self.tool == TOOL_BRUSH),
            ("3", "Box", self.tool == TOOL_RECT),
        ]:
            c = COLOR_TOOL_ACTIVE if active else default
            t = self._font.render(f"{key} {label}", True, c)
            self.screen.blit(t, (6, y))
            y += line_h
        y += gap
        # Terrains
        for key, label, active in [
            ("A", "Obst", self.terrain == TERRAIN_OBSTACLE),
            ("S", "Drive", self.terrain == TERRAIN_DRIVABLE),
            ("D", "Cut", self.terrain == TERRAIN_CUT),
        ]:
            c = COLOR_TOOL_ACTIVE if active else default
            t = self._font.render(f"{key} {label}", True, c)
            self.screen.blit(t, (6, y))
            y += line_h
        # Commands — bottom left of sidebar
        cmd_y = self.height - 5 * line_h - 8
        for key, label, active in [("W", "Goal", self.tool == TOOL_GOAL), ("E", "Path", self.tool == TOOL_PATH)]:
            c = COLOR_TOOL_ACTIVE if active else default
            t = self._font.render(f"{key} {label}", True, c)
            self.screen.blit(t, (6, cmd_y))
            cmd_y += line_h
        now = time.time()
        if now < self._save_flash_until:
            t = self._font.render("Saved", True, COLOR_TOOL_ACTIVE)
        else:
            t = self._font.render("Ctrl+S Save", True, default)
        self.screen.blit(t, (6, cmd_y))
        cmd_y += line_h
        t = self._font.render("Ctrl+Z Undo", True, default)
        self.screen.blit(t, (6, cmd_y))
        cmd_y += line_h
        t = self._font.render("Esc Close", True, default)
        self.screen.blit(t, (6, cmd_y))

    def _draw_tool_bar(self) -> None:
        """Draw sidebar (tools, terrain, save/undo)."""
        self._draw_sidebar()

    def _preview_color(self) -> tuple[int, int, int]:
        """Color for preview (rect/polygon) based on current terrain mode."""
        if self.terrain == TERRAIN_OBSTACLE:
            return COLOR_OBSTACLE_PREVIEW
        if self.terrain == TERRAIN_DRIVABLE:
            return COLOR_DRIVABLE_PREVIEW
        if self.terrain == TERRAIN_CUT:
            return COLOR_CUT_PREVIEW
        return COLOR_BG

    def _render(self) -> None:
        """Full frame: grid, drivable (with holes as BG), obstacles (with holes), hole-fill from base (excluding polygon interiors so draw-in-hole stays visible), brush overlays, preview, goals/path, robot, sidebar."""
        if self.background is not None:
            self.screen.blit(self.background, (0, 0))
            if self._grid_overlay is not None:
                self.screen.blit(self._grid_overlay, (0, 0))
        else:
            self._draw_grid(self.screen)
        for (ext, ints) in self.drivable:
            if len(ext) < 3:
                continue
            pts = [self._map_pixel_to_screen(p[0], p[1]) for p in ext]
            pygame.draw.polygon(self.screen, COLOR_DRIVABLE, [(int(x), int(y)) for x, y in pts])
            pygame.draw.polygon(self.screen, (200, 200, 200), [(int(x), int(y)) for x, y in pts], 1)
            for hole in ints:
                if len(hole) >= 3:
                    pts_h = [self._map_pixel_to_screen(p[0], p[1]) for p in hole]
                    pygame.draw.polygon(self.screen, COLOR_BG, [(int(x), int(y)) for x, y in pts_h])
        for (ext, ints) in self.obstacles:
            if len(ext) < 3:
                continue
            pts = [self._map_pixel_to_screen(p[0], p[1]) for p in ext]
            pygame.draw.polygon(self.screen, COLOR_OBSTACLE, [(int(x), int(y)) for x, y in pts])
            pygame.draw.polygon(self.screen, (220, 100, 100), [(int(x), int(y)) for x, y in pts], 1)
            for hole in ints:
                if len(hole) >= 3:
                    pts_h = [self._map_pixel_to_screen(p[0], p[1]) for p in hole]
                    pygame.draw.polygon(self.screen, COLOR_BG, [(int(x), int(y)) for x, y in pts_h])
        base_surf = self._bg_surface if self._bg_surface is not None else self.background
        if base_surf is not None:
            # Hole mask: fill only "empty" hole pixels with grid, not regions covered by another polygon.
            # So we can draw (brush/polygon) inside a hole and it stays visible (not overwritten by grid).
            mask_surf = pygame.Surface((self.width, self.height))
            mask_surf.fill((0, 0, 0))
            for (ext, ints) in self.drivable:
                for hole in ints:
                    if len(hole) >= 3:
                        pts_h = [self._map_pixel_to_screen(p[0], p[1]) for p in hole]
                        pygame.draw.polygon(mask_surf, (255, 255, 255), [(int(x), int(y)) for x, y in pts_h])
            for (ext, ints) in self.obstacles:
                for hole in ints:
                    if len(hole) >= 3:
                        pts_h = [self._map_pixel_to_screen(p[0], p[1]) for p in hole]
                        pygame.draw.polygon(mask_surf, (255, 255, 255), [(int(x), int(y)) for x, y in pts_h])
            # Punch out each polygon's *filled* region (exterior minus its holes) so that polygons
            # drawn inside a hole (e.g. drivable island) are not overwritten by the grid fill.
            for (ext, ints) in self.drivable:
                if len(ext) >= 3:
                    pts = [self._map_pixel_to_screen(p[0], p[1]) for p in ext]
                    pygame.draw.polygon(mask_surf, (0, 0, 0), [(int(x), int(y)) for x, y in pts])
                    for hole in ints:
                        if len(hole) >= 3:
                            pts_h = [self._map_pixel_to_screen(p[0], p[1]) for p in hole]
                            pygame.draw.polygon(mask_surf, (255, 255, 255), [(int(x), int(y)) for x, y in pts_h])
            for (ext, ints) in self.obstacles:
                if len(ext) >= 3:
                    pts = [self._map_pixel_to_screen(p[0], p[1]) for p in ext]
                    pygame.draw.polygon(mask_surf, (0, 0, 0), [(int(x), int(y)) for x, y in pts])
                    for hole in ints:
                        if len(hole) >= 3:
                            pts_h = [self._map_pixel_to_screen(p[0], p[1]) for p in hole]
                            pygame.draw.polygon(mask_surf, (255, 255, 255), [(int(x), int(y)) for x, y in pts_h])
            mask_arr = np.transpose(pygame.surfarray.array3d(mask_surf), (1, 0, 2))[:, :, 0] > 128
            if np.any(mask_arr):
                screen_arr = np.transpose(pygame.surfarray.array3d(self.screen), (1, 0, 2)).copy()
                bg_arr = np.transpose(pygame.surfarray.array3d(base_surf), (1, 0, 2))
                for c in range(3):
                    screen_arr[:, :, c] = np.where(mask_arr, bg_arr[:, :, c], screen_arr[:, :, c])
                pygame.surfarray.blit_array(self.screen, np.transpose(screen_arr, (1, 0, 2)))
        if self.drivable_brush_surface is not None:
            self.screen.blit(self.drivable_brush_surface, (0, 0))
        if self.brush_surface is not None:
            self.screen.blit(self.brush_surface, (0, 0))
        if self.erase_brush_surface is not None:
            self.screen.blit(self.erase_brush_surface, (0, 0))
        prev = self._preview_color()
        if self.rect_start is not None and self.rect_end is not None:
            sx0, sy0 = self._map_pixel_to_screen(*self.rect_start)
            sx1, sy1 = self._map_pixel_to_screen(*self.rect_end)
            r = pygame.Rect(min(sx0, sx1), min(sy0, sy1), abs(sx1 - sx0), abs(sy1 - sy0))
            pygame.draw.rect(self.screen, prev, r, 2)
        if self.current_polygon:
            pts = [self._map_pixel_to_screen(p[0], p[1]) for p in self.current_polygon]
            if len(pts) >= 2:
                pygame.draw.lines(self.screen, prev, False, pts, 2)
            for p in pts:
                pygame.draw.circle(self.screen, prev, (int(p[0]), int(p[1])), 4)
        for g in self.goals:
            sx, sy = self._map_pixel_to_screen(g[0], g[1])
            pygame.draw.circle(self.screen, COLOR_GOAL, (int(sx), int(sy)), GOAL_RADIUS)
            pygame.draw.circle(self.screen, (60, 180, 60), (int(sx), int(sy)), GOAL_RADIUS, 2)
        if len(self.path) >= 2:
            pts = [self._map_pixel_to_screen(p[0], p[1]) for p in self.path]
            pygame.draw.lines(self.screen, COLOR_PATH, False, pts, 3)
        if len(self.current_path) >= 2:
            pts = [self._map_pixel_to_screen(p[0], p[1]) for p in self.current_path]
            pygame.draw.lines(self.screen, COLOR_PATH, False, pts, 2)
        for p in self.path + self.current_path:
            sx, sy = self._map_pixel_to_screen(p[0], p[1])
            pygame.draw.circle(self.screen, COLOR_PATH, (int(sx), int(sy)), 5)
        now = time.time()
        if now - self._pose_cache_time >= POSE_POLL_INTERVAL:
            self._pose_cache = self.pose_getter()
            self._pose_cache_time = now
        pose = self._pose_cache
        if pose is not None:
            x, y, yaw = pose
            wx, wy = (y, x) if self._pose_swap_xy else (x, y)
            px_off, py_off = map_utils.world_to_pixel(
                wx,
                wy,
                self.origin[0],
                self.origin[1],
                self.scale_x,
                flip_y=self._pose_pixel_flip_y,
                scale_y=self.scale_y,
            )
            sx, sy = self._map_pixel_to_screen(px_off, py_off)
            yaw_rad = _robot_display_yaw_rad(yaw, self._pose_yaw_offset_deg, self._pose_swap_xy)
            _draw_robot_triangle(self.screen, (sx, sy), yaw_rad)
        self._draw_tool_bar()
        pygame.display.flip()

    def _compose_map_image(self) -> np.ndarray:
        """Build export image: base (bg or grid), then drivable (with holes black), brush, obstacles (with holes black). Order ensures brush and polys drawn in holes appear correctly."""
        if self.background is not None:
            base = pygame.surfarray.array3d(self.background)
        elif self._bg_surface is not None:
            base = pygame.surfarray.array3d(self._bg_surface)
        else:
            base = np.zeros((self.width, self.height, 3), dtype=np.uint8)
            base[:] = COLOR_BG
        base = np.transpose(base, (1, 0, 2)).copy()
        temp = pygame.Surface((self.width, self.height))
        temp.fill((0, 0, 0))
        temp.set_colorkey((0, 0, 0))
        for (ext, ints) in self.drivable:
            if len(ext) >= 3:
                pts = [self._map_pixel_to_screen(p[0], p[1]) for p in ext]
                pygame.draw.polygon(temp, COLOR_DRIVABLE, [(int(x), int(y)) for x, y in pts])
                for hole in ints:
                    if len(hole) >= 3:
                        pts_h = [self._map_pixel_to_screen(p[0], p[1]) for p in hole]
                        pygame.draw.polygon(temp, (0, 0, 0), [(int(x), int(y)) for x, y in pts_h])
        if self.drivable_brush_surface is not None:
            temp.blit(self.drivable_brush_surface, (0, 0))
        for (ext, ints) in self.obstacles:
            if len(ext) >= 3:
                pts = [self._map_pixel_to_screen(p[0], p[1]) for p in ext]
                pygame.draw.polygon(temp, COLOR_OBSTACLE, [(int(x), int(y)) for x, y in pts])
                for hole in ints:
                    if len(hole) >= 3:
                        pts_h = [self._map_pixel_to_screen(p[0], p[1]) for p in hole]
                        pygame.draw.polygon(temp, (0, 0, 0), [(int(x), int(y)) for x, y in pts_h])
        if self.brush_surface is not None:
            temp.blit(self.brush_surface, (0, 0))
        drawn = np.transpose(pygame.surfarray.array3d(temp), (1, 0, 2))
        drawn_mask = (drawn[:, :, 0] > 0) | (drawn[:, :, 1] > 0) | (drawn[:, :, 2] > 0)
        for c in range(3):
            base[:, :, c] = np.where(drawn_mask, drawn[:, :, c], base[:, :, c])
        return base

    def _save(self, path_prefix: str) -> None:
        """Write map image (composed) and JSON (obstacles, drivable with exteriors/interiors, goals, path). erase_polygons not persisted (cuts are baked into terrain)."""
        meta = dict(self.metadata)
        meta["origin"] = [float(self.origin[0]), float(self.origin[1])]
        meta["scale"] = self.scale_x
        meta["scale_x"] = self.scale_x
        meta["scale_y"] = self.scale_y
        meta["pose_pixel_flip_y"] = self._pose_pixel_flip_y
        meta["pose_swap_xy"] = self._pose_swap_xy
        meta["pose_yaw_offset_deg"] = self._pose_yaw_offset_deg
        meta["map_mirror_x"] = self._map_mirror_x
        meta["map_mirror_y"] = self._map_mirror_y
        if self._capture_native_w > 0 and self._capture_native_h > 0:
            meta["capture_native_width"] = self._capture_native_w
            meta["capture_native_height"] = self._capture_native_h
        meta["goals"] = [[float(p[0]), float(p[1])] for p in self.goals]
        meta["path"] = [[float(p[0]), float(p[1])] for p in self.path]
        meta["obstacles"] = [
            {"exterior": [[float(x), float(y)] for x, y in ext], "interiors": [[[float(x), float(y)] for x, y in h] for h in ints]}
            for (ext, ints) in self.obstacles
        ]
        meta["drivable"] = [
            {"exterior": [[float(x), float(y)] for x, y in ext], "interiors": [[[float(x), float(y)] for x, y in h] for h in ints]}
            for (ext, ints) in self.drivable
        ]
        meta["erase_polygons"] = []
        map_utils.save_map(path_prefix, self._compose_map_image(), meta)
        self._save_flash_until = time.time() + 0.6
        print(f"Saved {path_prefix}.png and {path_prefix}.json")

    def run(self, save_path_prefix: str | None = None) -> None:
        """Main loop: events (click for goal/path/polygon/rect/brush, close polygon on repeat click or RMB, Ctrl+S save, Ctrl+Z undo, Esc cancel or quit), then _render."""
        save_path_prefix = save_path_prefix or "data/maps/map"
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.VIDEORESIZE:
                    self.width, self.height = event.w, event.h
                    self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
                    self.view_center = (self.width // 2, self.height // 2)
                    self._make_brush_surfaces()
                    self._build_bg_surface()
                    self._build_grid_overlay()
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        mx, my = self._screen_to_map_pixel(*event.pos)
                        if self.tool == TOOL_GOAL:
                            self._push_undo_state()
                            self.goals.append((mx, my))
                        elif self.tool == TOOL_PATH:
                            if len(self.current_path) >= 2:
                                sx0, sy0 = self._map_pixel_to_screen(self.current_path[0][0], self.current_path[0][1])
                                if (event.pos[0] - sx0) ** 2 + (event.pos[1] - sy0) ** 2 <= CLOSE_POINT_RADIUS ** 2:
                                    self._push_undo_state()
                                    self.path = self.current_path.copy()
                                    self.current_path.clear()
                                else:
                                    self.current_path.append((mx, my))
                            else:
                                self.current_path.append((mx, my))
                        elif self.tool == TOOL_POLYGON:
                            if len(self.current_polygon) >= 3:
                                for pt in self.current_polygon:
                                    sx, sy = self._map_pixel_to_screen(pt[0], pt[1])
                                    if (event.pos[0] - sx) ** 2 + (event.pos[1] - sy) ** 2 <= CLOSE_POINT_RADIUS ** 2:
                                        self._commit_polygon()
                                        self.current_polygon.clear()
                                        break
                                else:
                                    self.current_polygon.append((mx, my))
                            else:
                                self.current_polygon.append((mx, my))
                        elif self.tool == TOOL_RECT:
                            self.rect_start = self.rect_end = (mx, my)
                        elif self.tool == TOOL_BRUSH:
                            self._brush_draw(event.pos)
                    elif event.button == 3:
                        if self.tool == TOOL_POLYGON and self.current_polygon:
                            if len(self.current_polygon) >= 3:
                                self._commit_polygon()
                            self.current_polygon.clear()
                        elif self.tool == TOOL_RECT:
                            self.rect_start = self.rect_end = None
                        elif self.tool == TOOL_PATH and self.current_path:
                            if len(self.current_path) >= 2:
                                self._push_undo_state()
                                self.path = self.current_path.copy()
                            self.current_path.clear()
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1 and self.tool == TOOL_RECT and self.rect_start and self.rect_end:
                        self._commit_rect()
                elif event.type == pygame.MOUSEMOTION:
                    if event.buttons[0]:
                        if self.tool == TOOL_BRUSH:
                            self._brush_draw(event.pos)
                        elif self.tool == TOOL_RECT and self.rect_start is not None:
                            self.rect_end = self._screen_to_map_pixel(*event.pos)
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.current_polygon.clear()
                        self.rect_start = self.rect_end = None
                        self.current_path.clear()
                        if not self.current_polygon and self.rect_start is None:
                            self.running = False
                    mod = pygame.key.get_mods()
                    ctrl_or_cmd = mod & (pygame.KMOD_CTRL | pygame.KMOD_META)
                    if event.key == pygame.K_s and ctrl_or_cmd:
                        self._save(save_path_prefix)
                    elif event.key == pygame.K_z and ctrl_or_cmd:
                        self._undo()
                    elif event.key == pygame.K_1:
                        self.tool = TOOL_POLYGON
                    elif event.key == pygame.K_2:
                        self.tool = TOOL_BRUSH
                    elif event.key == pygame.K_3:
                        self.tool = TOOL_RECT
                    elif event.key == pygame.K_a:
                        self.terrain = TERRAIN_OBSTACLE
                    elif event.key == pygame.K_s:
                        self.terrain = TERRAIN_DRIVABLE
                    elif event.key == pygame.K_d:
                        self.terrain = TERRAIN_CUT
                    elif event.key == pygame.K_w:
                        self.tool = TOOL_GOAL
                    elif event.key == pygame.K_e:
                        self.tool = TOOL_PATH
                    elif event.key == pygame.K_RETURN:
                        if self.tool == TOOL_POLYGON and self.current_polygon and len(self.current_polygon) >= 3:
                            self._commit_polygon()
                            self.current_polygon.clear()
                        elif self.tool == TOOL_PATH and len(self.current_path) >= 2:
                            self.path = self.current_path.copy()
                            self.current_path.clear()
            self._render()
            self.clock.tick(60)
        pygame.quit()
