# Path sampling for data collection.
#
# - **PolylinePath**: piecewise linear between JSON control points (chords only).
# - **CatmullRomPath**: smooth curve *through* every control point (C1 Catmull-Rom),
#   with arc-length built from a dense polyline along the analytic curve. This
#   approximates the "line between points" you get from Unreal's
#   `USplineComponent` + auto tangents better than chord-only sampling, and
#   yields many intermediate poses when stepped by arc length.
#
# Unreal's exact spline tangents can differ slightly; use `--curve polyline`
# if you need strict chord following.

from __future__ import annotations

import bisect
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Sequence

CurveMode = Literal["polyline", "catmull_rom"]


@dataclass(frozen=True)
class SplineSample:
    x: float
    y: float
    yaw_deg: float  # heading (deg, CCW from +X), UE-style
    s: float        # arc length from start along the chosen curve
    segment_index: int


def _cr_point(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    """Catmull-Rom: segment runs from p1 to p2 with ghost controls p0, p3."""
    t2 = t * t
    t3 = t2 * t
    x = 0.5 * (
        2 * p1[0]
        + (-p0[0] + p2[0]) * t
        + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
        + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3
    )
    y = 0.5 * (
        2 * p1[1]
        + (-p0[1] + p2[1]) * t
        + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
        + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3
    )
    return (x, y)


def _cr_deriv(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    """Derivative w.r.t. parameter t in [0, 1] on the p1-p2 segment."""
    return (
        0.5
        * (
            (-p0[0] + p2[0])
            + 2 * (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t
            + 3 * (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t * t
        ),
        0.5
        * (
            (-p0[1] + p2[1])
            + 2 * (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t
            + 3 * (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t * t
        ),
    )


class PolylinePath:
    """Piecewise linear path; arc length is exact along chords."""

    kind: CurveMode = "polyline"

    def __init__(self, points: Sequence[Sequence[float]]) -> None:
        pts = [(float(p[0]), float(p[1])) for p in points if len(p) >= 2]
        if len(pts) < 2:
            raise ValueError("Path needs at least 2 points")
        self._pts: list[tuple[float, float]] = pts
        self._cum: list[float] = [0.0]
        for i in range(1, len(pts)):
            dx = pts[i][0] - pts[i - 1][0]
            dy = pts[i][1] - pts[i - 1][1]
            self._cum.append(self._cum[-1] + math.hypot(dx, dy))

    @property
    def total_length(self) -> float:
        return self._cum[-1]

    @property
    def points(self) -> list[tuple[float, float]]:
        return list(self._pts)

    def sample(self, s: float) -> SplineSample:
        s = max(0.0, min(float(s), self.total_length))
        seg = self._find_segment(s)
        x0, y0 = self._pts[seg]
        x1, y1 = self._pts[seg + 1]
        seg_len = self._cum[seg + 1] - self._cum[seg]
        t = 0.0 if seg_len <= 0 else (s - self._cum[seg]) / seg_len
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)
        yaw = math.degrees(math.atan2(y1 - y0, x1 - x0))
        return SplineSample(x=x, y=y, yaw_deg=yaw, s=s, segment_index=seg)

    def sample_with_lookahead(self, s: float, lookahead: float) -> SplineSample:
        here = self.sample(s)
        ahead = self.sample(min(self.total_length, s + max(0.0, lookahead)))
        dx = ahead.x - here.x
        dy = ahead.y - here.y
        if dx == 0.0 and dy == 0.0:
            return here
        yaw = math.degrees(math.atan2(dy, dx))
        return SplineSample(
            x=here.x, y=here.y, yaw_deg=yaw, s=here.s, segment_index=here.segment_index
        )

    def _find_segment(self, s: float) -> int:
        lo, hi = 0, len(self._cum) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if self._cum[mid] <= s:
                lo = mid
            else:
                hi = mid
        return lo


class CatmullRomPath:
    """Smooth path through all control points; arc length from a dense polyline."""

    kind: CurveMode = "catmull_rom"

    def __init__(
        self,
        points: Sequence[Sequence[float]],
        *,
        samples_per_segment: int = 48,
    ) -> None:
        pts = [(float(p[0]), float(p[1])) for p in points if len(p) >= 2]
        if len(pts) < 2:
            raise ValueError("Path needs at least 2 points")
        self._pts = pts
        sp = max(4, int(samples_per_segment))
        self._samples_per_segment = sp

        self._arc_s: list[float] = [0.0]
        self._arc_seg: list[int] = [0]
        self._arc_u: list[float] = [0.0]
        self._arc_x: list[float] = [self._eval_seg(0, 0.0)[0]]
        self._arc_y: list[float] = [self._eval_seg(0, 0.0)[1]]
        px, py = self._arc_x[0], self._arc_y[0]
        s_acc = 0.0
        n = len(pts)
        for seg in range(n - 1):
            start_k = 0 if seg == 0 else 1
            for k in range(start_k, sp + 1):
                u = k / sp
                x, y = self._eval_seg(seg, u)
                s_acc += math.hypot(x - px, y - py)
                px, py = x, y
                self._arc_s.append(s_acc)
                self._arc_seg.append(seg)
                self._arc_u.append(u)
                self._arc_x.append(x)
                self._arc_y.append(y)
        self._total = s_acc

    def _ghost(self, i: int) -> tuple[float, float]:
        """Clamp indices for Catmull-Rom endpoints."""
        n = len(self._pts)
        if i < 0:
            return self._pts[0]
        if i >= n:
            return self._pts[n - 1]
        return self._pts[i]

    def _eval_seg(self, seg: int, u: float) -> tuple[float, float]:
        p0 = self._ghost(seg - 1)
        p1 = self._ghost(seg)
        p2 = self._ghost(seg + 1)
        p3 = self._ghost(seg + 2)
        return _cr_point(p0, p1, p2, p3, u)

    def _deriv_seg(self, seg: int, u: float) -> tuple[float, float]:
        p0 = self._ghost(seg - 1)
        p1 = self._ghost(seg)
        p2 = self._ghost(seg + 1)
        p3 = self._ghost(seg + 2)
        return _cr_deriv(p0, p1, p2, p3, u)

    @property
    def total_length(self) -> float:
        return self._total

    @property
    def points(self) -> list[tuple[float, float]]:
        return list(self._pts)

    def sample(self, s: float) -> SplineSample:
        s = max(0.0, min(float(s), self._total))
        if s <= 0.0:
            seg = self._arc_seg[0]
            u = self._arc_u[0]
            dx, dy = self._deriv_seg(seg, u)
            yaw = math.degrees(math.atan2(dy, dx))
            return SplineSample(
                x=self._arc_x[0],
                y=self._arc_y[0],
                yaw_deg=yaw,
                s=0.0,
                segment_index=seg,
            )
        if s >= self._total:
            seg = self._arc_seg[-1]
            u = self._arc_u[-1]
            dx, dy = self._deriv_seg(seg, u)
            yaw = math.degrees(math.atan2(dy, dx))
            return SplineSample(
                x=self._arc_x[-1],
                y=self._arc_y[-1],
                yaw_deg=yaw,
                s=self._total,
                segment_index=seg,
            )

        i = bisect.bisect_right(self._arc_s, s) - 1
        i = max(0, min(i, len(self._arc_s) - 2))
        s0, s1 = self._arc_s[i], self._arc_s[i + 1]
        alpha = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)

        if self._arc_seg[i] == self._arc_seg[i + 1]:
            seg = self._arc_seg[i]
            u = self._arc_u[i] + alpha * (self._arc_u[i + 1] - self._arc_u[i])
            u = max(0.0, min(1.0, u))
            x, y = self._eval_seg(seg, u)
            dx, dy = self._deriv_seg(seg, u)
        else:
            x = self._arc_x[i] + alpha * (self._arc_x[i + 1] - self._arc_x[i])
            y = self._arc_y[i] + alpha * (self._arc_y[i + 1] - self._arc_y[i])
            dx = self._arc_x[i + 1] - self._arc_x[i]
            dy = self._arc_y[i + 1] - self._arc_y[i]
            seg = self._arc_seg[i]

        yaw = math.degrees(math.atan2(dy, dx)) if (dx != 0.0 or dy != 0.0) else 0.0
        return SplineSample(x=x, y=y, yaw_deg=yaw, s=s, segment_index=seg)

    def sample_with_lookahead(self, s: float, lookahead: float) -> SplineSample:
        here = self.sample(s)
        ahead = self.sample(min(self._total, s + max(0.0, lookahead)))
        dx = ahead.x - here.x
        dy = ahead.y - here.y
        if dx == 0.0 and dy == 0.0:
            return here
        yaw = math.degrees(math.atan2(dy, dx))
        return SplineSample(
            x=here.x, y=here.y, yaw_deg=yaw, s=here.s, segment_index=here.segment_index
        )


# Backwards compatibility: old name referred to the polyline implementation.
Spline = PolylinePath


def _path_xy_from_world_points(meta: dict) -> list[list[float]] | None:
    """Build ``[[x, y], ...]`` from outline exports that use ``world_points`` instead of ``path``."""
    wp = meta.get("world_points")
    if not isinstance(wp, list) or len(wp) < 2:
        return None
    rows: list[tuple[int, float, float]] = []
    for i, item in enumerate(wp):
        if isinstance(item, dict) and "x" in item and "y" in item:
            idx = int(item["index"]) if item.get("index") is not None else i
            rows.append((idx, float(item["x"]), float(item["y"])))
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            rows.append((i, float(item[0]), float(item[1])))
    if len(rows) < 2:
        return None
    rows.sort(key=lambda t: t[0])
    return [[x, y] for _, x, y in rows]


def load_path_from_map(
    map_json_path: str | Path,
    *,
    key: str = "path",
    curve: CurveMode = "catmull_rom",
    samples_per_segment: int = 48,
) -> PolylinePath | CatmullRomPath:
    p = Path(map_json_path)
    with open(p) as f:
        meta = json.load(f)
    raw = meta.get(key)
    if not raw:
        raw = _path_xy_from_world_points(meta)
    if not raw:
        raise ValueError(
            f"{p} has no '{key}' entry and no usable 'world_points'. "
            f"Export a path from the UE5 Path Tape plugin / BoxSim map builder, "
            f"or include `world_points` with x/y per point."
        )
    space = str(meta.get("path_space", "world")).lower()
    if space not in ("world", "map_scaled"):
        raise ValueError(f"Unsupported path_space={space!r} in {p}")
    if space != "world":
        raise ValueError(
            f"path_space={space!r} in {p}: this pipeline expects world-space "
            f"coordinates. Re-export with path_space='world'."
        )
    if curve == "polyline":
        return PolylinePath(raw)
    if curve == "catmull_rom":
        return CatmullRomPath(raw, samples_per_segment=samples_per_segment)
    raise ValueError(f"Unknown curve mode: {curve!r}")


def load_spline_from_map(
    map_json_path: str | Path, *, key: str = "path"
) -> PolylinePath:
    """Load chord-only path (legacy / strict JSON edges). Prefer `load_path_from_map`."""
    return load_path_from_map(map_json_path, key=key, curve="polyline")


def load_goals_from_map(
    map_json_path: str | Path, *, key: str = "goals"
) -> list[tuple[float, float]]:
    """Load `goals` (list of [x, y] world points) from a BoxSim map JSON.

    Returns [] if the file has no goals key. Skips malformed entries.
    """
    p = Path(map_json_path)
    with open(p) as f:
        meta = json.load(f)
    raw = meta.get(key) or []
    out: list[tuple[float, float]] = []
    for entry in raw:
        if isinstance(entry, dict):
            if "x" in entry and "y" in entry:
                try:
                    out.append((float(entry["x"]), float(entry["y"])))
                except (TypeError, ValueError):
                    continue
            continue
        try:
            if len(entry) >= 2:
                out.append((float(entry[0]), float(entry[1])))
        except (TypeError, ValueError):
            continue
    return out
