# Map viewer. 1/2/3=shape, A=obstacle S=drivable D=cut, W=goal E=path, S=Save.

from __future__ import annotations

import math
import time
from typing import Callable

import pygame
import numpy as np

import map_utils

TOOL_POLYGON, TOOL_BRUSH, TOOL_RECT = "polygon", "brush", "rect"
TOOL_GOAL, TOOL_PATH = "goal", "path"
TERRAIN_OBSTACLE, TERRAIN_DRIVABLE, TERRAIN_CUT = "obstacle", "drivable", "cut"
COLOR_BG = (40, 44, 52)
COLOR_GRID = (60, 64, 72)
COLOR_OBSTACLE = (200, 60, 60)
COLOR_DRIVABLE = (240, 240, 240)
COLOR_OBSTACLE_PREVIEW = (220, 100, 100)
COLOR_DRIVABLE_PREVIEW = (200, 200, 200)
COLOR_GOAL = (80, 220, 80)
COLOR_PATH = (255, 220, 80)
COLOR_ROBOT = (100, 180, 100)
COLOR_ROBOT_OUTLINE = (60, 120, 60)
COLOR_AXIS = (100, 100, 120)
BRUSH_RADIUS = 8
GRID_SIZE = 50
POSE_POLL_INTERVAL = 0.2
CLOSE_POINT_RADIUS = 12
COLOR_TOOL_ACTIVE = (120, 255, 120)
GOAL_RADIUS = 10
COORD_TICK_INTERVAL = 100


def _world_to_screen(world_x: float, world_y: float, origin: tuple[float, float],
                     scale: float, view_center: tuple[float, float]) -> tuple[float, float]:
    px, py = map_utils.world_to_pixel(world_x, world_y, origin[0], origin[1], scale, flip_y=True)
    return (view_center[0] + px, view_center[1] - py)


def _draw_robot_triangle(surface: pygame.Surface, center: tuple[float, float],
                         yaw_deg: float, size: float = 12) -> None:
    yaw_rad = math.radians(yaw_deg) + math.pi
    nose = (center[0] + size * math.cos(yaw_rad), center[1] - size * math.sin(yaw_rad))
    back = yaw_rad + math.pi
    half = size * 0.6
    bl = (center[0] + half * math.cos(back + 0.4), center[1] - half * math.sin(back + 0.4))
    br = (center[0] + half * math.cos(back - 0.4), center[1] - half * math.sin(back - 0.4))
    pts = [nose, bl, br]
    pygame.draw.polygon(surface, COLOR_ROBOT, pts)
    pygame.draw.polygon(surface, COLOR_ROBOT_OUTLINE, pts, 2)


class MapViewer:
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
        self.width, self.height = width, height
        self.metadata = dict(metadata)
        self.pose_getter = pose_getter
        self.world_width = world_width or self.metadata.get("ortho_width", 4000)
        self.world_height = world_height or self.metadata.get("world_height") or self.world_width
        self.origin = tuple(self.metadata.get("origin", [0, 0]))
        self.scale = self.metadata.get("scale", 0.2)
        self.obstacles: list[list[tuple[float, float]]] = []
        self.drivable: list[list[tuple[float, float]]] = []
        self.erase_polygons: list[list[tuple[float, float]]] = []
        self.current_polygon: list[tuple[float, float]] = []
        self.rect_start = self.rect_end = None
        self.brush_surface = None
        self.drivable_brush_surface = None
        self.erase_brush_surface = None
        self.tool = TOOL_POLYGON
        self.terrain = TERRAIN_OBSTACLE
        self.goals: list[tuple[float, float]] = []
        self.path: list[tuple[float, float]] = []
        self.current_path: list[tuple[float, float]] = []
        self.view_offset_x = self.view_offset_y = 0.0

        if background_image is not None:
            if background_image.ndim == 2:
                bg = pygame.surfarray.make_surface(np.stack([background_image] * 3, axis=-1))
            else:
                bg = pygame.surfarray.make_surface(np.transpose(background_image, (1, 0, 2)))
            self.background = pygame.transform.scale(bg, (width, height))
        else:
            self.background = None

        self.view_center = (width // 2, height // 2)
        pygame.init()
        self.screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
        pygame.display.set_caption("Map Builder")
        self.clock = pygame.time.Clock()
        self._make_brush_surfaces()
        self.running = True
        self._pose_cache: tuple[float, float, float] | None = None
        self._pose_cache_time = 0.0
        self._font = pygame.font.Font(None, 24)
        self._save_flash_until = 0.0

    def _make_brush_surfaces(self) -> None:
        for name in ("brush_surface", "drivable_brush_surface", "erase_brush_surface"):
            s = pygame.Surface((self.width, self.height))
            s.set_colorkey((0, 0, 0))
            s.fill((0, 0, 0))
            setattr(self, name, s)

    def _draw_grid(self, surface: pygame.Surface) -> None:
        surface.fill(COLOR_BG)
        for x in range(0, self.width + 1, GRID_SIZE):
            pygame.draw.line(surface, COLOR_GRID, (x, 0), (x, self.height))
        for y in range(0, self.height + 1, GRID_SIZE):
            pygame.draw.line(surface, COLOR_GRID, (0, y), (self.width, y))
        self._draw_coord_overlay(surface)

    def _draw_coord_overlay(self, surface: pygame.Surface) -> None:
        vc = (self.view_center[0] + self.view_offset_x, self.view_center[1] + self.view_offset_y)
        lo, hi = -500, 500
        step = COORD_TICK_INTERVAL
        for wx in range(lo, hi + 1, step):
            mx = wx * self.scale
            px, _ = self._map_pixel_to_screen(mx, 0)
            if -50 <= px < self.width + 50:
                w = 2 if wx == 0 else 1
                pygame.draw.line(surface, COLOR_AXIS, (int(px), 0), (int(px), self.height), w)
                t = self._font.render(str(wx), True, COLOR_AXIS)
                surface.blit(t, (int(px) - t.get_width() // 2, int(vc[1]) + 4))
        for wy in range(lo, hi + 1, step):
            my = wy * self.scale
            _, py = self._map_pixel_to_screen(0, my)
            if -50 <= py < self.height + 50:
                w = 2 if wy == 0 else 1
                pygame.draw.line(surface, COLOR_AXIS, (0, int(py)), (self.width, int(py)), w)
                t = self._font.render(str(-wy), True, COLOR_AXIS)
                surface.blit(t, (int(vc[0]) - t.get_width() - 4, int(py) - t.get_height() // 2))

    def _screen_to_map_pixel(self, sx: float, sy: float) -> tuple[float, float]:
        mx = sx - self.view_center[0] - self.view_offset_x
        my = -(sy - self.view_center[1] - self.view_offset_y)
        return (mx, my)

    def _map_pixel_to_screen(self, mx: float, my: float) -> tuple[float, float]:
        return (self.view_center[0] + mx + self.view_offset_x,
                self.view_center[1] - my + self.view_offset_y)

    def _commit_polygon(self) -> None:
        if len(self.current_polygon) < 3:
            return
        poly = self.current_polygon.copy()
        if self.terrain == TERRAIN_OBSTACLE:
            self.obstacles.append(poly)
        elif self.terrain == TERRAIN_DRIVABLE:
            self.drivable.append(poly)
        else:
            self.erase_polygons.append(poly)

    def _commit_rect(self) -> None:
        if not self.rect_start or not self.rect_end:
            return
        x0, y0, x1, y1 = *self.rect_start, *self.rect_end
        poly = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        if self.terrain == TERRAIN_OBSTACLE:
            self.obstacles.append(poly)
        elif self.terrain == TERRAIN_DRIVABLE:
            self.drivable.append(poly)
        else:
            self.erase_polygons.append(poly)
        self.rect_start = self.rect_end = None

    def _brush_draw(self, pos: tuple[int, int]) -> None:
        if self.terrain == TERRAIN_OBSTACLE and self.brush_surface:
            pygame.draw.circle(self.brush_surface, COLOR_OBSTACLE, pos, BRUSH_RADIUS)
        elif self.terrain == TERRAIN_DRIVABLE and self.drivable_brush_surface:
            pygame.draw.circle(self.drivable_brush_surface, COLOR_DRIVABLE, pos, BRUSH_RADIUS)
        elif self.terrain == TERRAIN_CUT and self.erase_brush_surface:
            pygame.draw.circle(self.erase_brush_surface, COLOR_BG, pos, BRUSH_RADIUS)

    def _draw_tool_bar(self) -> None:
        now = time.time()
        default = (200, 200, 200)
        items = [
            ("1", "Poly", self.tool == TOOL_POLYGON),
            ("2", "Brush", self.tool == TOOL_BRUSH),
            ("3", "Box", self.tool == TOOL_RECT),
            ("A", "Obst", self.terrain == TERRAIN_OBSTACLE),
            ("S", "Drive", self.terrain == TERRAIN_DRIVABLE),
            ("D", "Cut", self.terrain == TERRAIN_CUT),
            ("W", "Goal", self.tool == TOOL_GOAL),
            ("E", "Path", self.tool == TOOL_PATH),
            ("Ctrl+S", "Save", now < self._save_flash_until),
        ]
        x = 8
        for key, label, active in items:
            c = COLOR_TOOL_ACTIVE if active else default
            t = self._font.render(f"{key}={label}", True, c)
            self.screen.blit(t, (x, self.height - 24))
            x += t.get_width() + 8

    def _preview_color(self) -> tuple[int, int, int]:
        if self.terrain == TERRAIN_OBSTACLE:
            return COLOR_OBSTACLE_PREVIEW
        if self.terrain == TERRAIN_DRIVABLE:
            return COLOR_DRIVABLE_PREVIEW
        return COLOR_BG

    def _render(self) -> None:
        if self.background is not None:
            self.screen.blit(self.background, (0, 0))
        else:
            self._draw_grid(self.screen)
        for poly in self.drivable:
            if len(poly) < 3:
                continue
            pts = [self._map_pixel_to_screen(p[0], p[1]) for p in poly]
            pygame.draw.polygon(self.screen, COLOR_DRIVABLE, pts)
            pygame.draw.polygon(self.screen, (200, 200, 200), pts, 1)
        for poly in self.obstacles:
            if len(poly) < 3:
                continue
            pts = [self._map_pixel_to_screen(p[0], p[1]) for p in poly]
            pygame.draw.polygon(self.screen, COLOR_OBSTACLE, pts)
            pygame.draw.polygon(self.screen, (220, 100, 100), pts, 1)
        for poly in self.erase_polygons:
            if len(poly) < 3:
                continue
            pts = [self._map_pixel_to_screen(p[0], p[1]) for p in poly]
            pygame.draw.polygon(self.screen, COLOR_BG, pts)
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
            vc = (self.view_center[0] + self.view_offset_x, self.view_center[1] + self.view_offset_y)
            sx, sy = _world_to_screen(x, y, self.origin, self.scale, vc)
            _draw_robot_triangle(self.screen, (sx, sy), yaw)
        self._draw_tool_bar()
        pygame.display.flip()

    def _compose_map_image(self) -> np.ndarray:
        if self.background is not None:
            out = np.transpose(pygame.surfarray.array3d(self.background), (1, 0, 2))
        else:
            out = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            out[:] = COLOR_BG
        temp = pygame.Surface((self.width, self.height))
        temp.fill((0, 0, 0))
        temp.set_colorkey((0, 0, 0))
        for poly in self.drivable:
            if len(poly) >= 3:
                pts = [self._map_pixel_to_screen(p[0], p[1]) for p in poly]
                pygame.draw.polygon(temp, COLOR_DRIVABLE, pts)
        if self.drivable_brush_surface is not None:
            temp.blit(self.drivable_brush_surface, (0, 0))
        for poly in self.obstacles:
            if len(poly) >= 3:
                pts = [self._map_pixel_to_screen(p[0], p[1]) for p in poly]
                pygame.draw.polygon(temp, COLOR_OBSTACLE, pts)
        if self.brush_surface is not None:
            temp.blit(self.brush_surface, (0, 0))
        for poly in self.erase_polygons:
            if len(poly) >= 3:
                pts = [self._map_pixel_to_screen(p[0], p[1]) for p in poly]
                pygame.draw.polygon(temp, COLOR_BG, pts)
        if self.erase_brush_surface is not None:
            temp.blit(self.erase_brush_surface, (0, 0))
        arr = np.transpose(pygame.surfarray.array3d(temp), (1, 0, 2))
        mask = (arr[:, :, 0] > 0) | (arr[:, :, 1] > 0) | (arr[:, :, 2] > 0)
        for c in range(3):
            out[:, :, c] = np.where(mask, arr[:, :, c], out[:, :, c])
        return out

    def _save(self, path_prefix: str) -> None:
        meta = dict(self.metadata)
        meta["goals"] = [[float(p[0]), float(p[1])] for p in self.goals]
        meta["path"] = [[float(p[0]), float(p[1])] for p in self.path]
        map_utils.save_map(path_prefix, self._compose_map_image(), meta)
        self._save_flash_until = time.time() + 0.6
        print(f"Saved {path_prefix}.png and {path_prefix}.json")

    def run(self, save_path_prefix: str | None = None) -> None:
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
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        mx, my = self._screen_to_map_pixel(*event.pos)
                        if self.tool == TOOL_GOAL:
                            self.goals.append((mx, my))
                        elif self.tool == TOOL_PATH:
                            if len(self.current_path) >= 2:
                                sx0, sy0 = self._map_pixel_to_screen(self.current_path[0][0], self.current_path[0][1])
                                if (event.pos[0] - sx0) ** 2 + (event.pos[1] - sy0) ** 2 <= CLOSE_POINT_RADIUS ** 2:
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
                    elif event.key == pygame.K_s and (pygame.key.get_mods() & pygame.KMOD_CTRL):
                        self._save(save_path_prefix)
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
