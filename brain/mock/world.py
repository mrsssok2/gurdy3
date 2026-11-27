"""2D world for the mock backend: rooms, walls, obstacles, ground-truth objects, occupancy grid, A*.

`flat_lab()` mirrors scripts/gen_world_flat_lab.py (two rooms, doorway at x=2.5, y in [0, 1]); the objects use the
detector's class names (the Gazebo flat_lab has plain boxes / a cylinder, so the classes here are an adaptation).
Pure Python, no numpy.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

RES = 0.1            # grid cell, m
ROBOT_RADIUS = 0.25  # planning inflation (footprint + margin)


def zone_hit(z, x: float, y: float) -> bool:
    """A no-go zone as a tuple: circle (x, y, r) or rectangle (x0, y0, x1, y1)."""
    if len(z) == 4:
        return z[0] <= x <= z[2] and z[1] <= y <= z[3]
    return math.hypot(x - z[0], y - z[1]) <= z[2]


@dataclass
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float

    def contains(self, x: float, y: float) -> bool:
        return self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1

    @property
    def area(self) -> float:
        return (self.x1 - self.x0) * (self.y1 - self.y0)

    @property
    def centre(self) -> tuple[float, float]:
        return ((self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2)


@dataclass
class GtObject:
    gid: int
    cls: str
    color: str
    x: float
    y: float
    radius: float = 0.2
    present: bool = True


@dataclass
class World:
    bounds: Rect
    rooms: dict[str, Rect]
    walls: list[Rect]
    obstacles: list[Rect] = field(default_factory=list)
    objects: list[GtObject] = field(default_factory=list)
    places: dict[str, tuple[float, float]] = field(default_factory=dict)
    hazards: set[str] = field(default_factory=set)
    start: tuple[float, float, float] = (0.0, 0.0, 0.0)   # x, y, yaw
    map_id: str = "mock_flat_lab"

    def __post_init__(self) -> None:
        self._build()

    # ---------------------------------------------------------------- grid
    def _cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(math.floor((x - self.bounds.x0) / RES)), int(math.floor((y - self.bounds.y0) / RES)))

    def _centre(self, c: tuple[int, int]) -> tuple[float, float]:
        return (self.bounds.x0 + (c[0] + 0.5) * RES, self.bounds.y0 + (c[1] + 0.5) * RES)

    def _raster_rect(self, r: Rect) -> set[tuple[int, int]]:
        a, b = self._cell(r.x0, r.y0), self._cell(r.x1, r.y1)
        return {(i, j) for i in range(a[0], b[0] + 1) for j in range(a[1], b[1] + 1)}

    def _raster_disc(self, x: float, y: float, rad: float) -> set[tuple[int, int]]:
        n = int(math.ceil(rad / RES))
        c = self._cell(x, y)
        return {(c[0] + i, c[1] + j) for i in range(-n, n + 1) for j in range(-n, n + 1)
                if math.hypot(i * RES, j * RES) <= rad}

    def _inflate(self, cells: set[tuple[int, int]], rad: float) -> set[tuple[int, int]]:
        n = int(math.ceil(rad / RES))
        ring = [(i, j) for i in range(-n, n + 1) for j in range(-n, n + 1) if math.hypot(i * RES, j * RES) <= rad]
        return {(c[0] + i, c[1] + j) for c in cells for i, j in ring}

    def _build(self) -> None:
        """(Re)build the occupancy sets. Call after changing obstacles / objects."""
        hard: set[tuple[int, int]] = set()
        for r in (*self.walls, *self.obstacles):
            hard |= self._raster_rect(r)
        self.sight_blockers = hard                      # what blocks the camera: walls + obstacles, not objects
        objs: set[tuple[int, int]] = set()
        for o in self.objects:
            if o.present:
                objs |= self._raster_disc(o.x, o.y, o.radius)
        self.blocked_cells = self._inflate(hard | objs, ROBOT_RADIUS)

    # ---------------------------------------------------------------- queries
    def place_at(self, x: float, y: float) -> str:
        for name, (px, py) in self.places.items():
            if name == "doorway" and math.hypot(x - px, y - py) < 0.7:
                return "doorway"
        for name, r in self.rooms.items():
            if r.contains(x, y):
                return name
        return "outside"

    def is_free(self, x: float, y: float) -> bool:
        return self.bounds.contains(x, y) and self._cell(x, y) not in self.blocked_cells

    def line_free(self, p: tuple[float, float], q: tuple[float, float], zones=()) -> bool:
        n = max(2, int(math.hypot(q[0] - p[0], q[1] - p[1]) / 0.0125))
        for k in range(1, n + 1):
            x, y = p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n
            if not self.is_free(x, y) or any(zone_hit(z, x, y) for z in zones):
                return False
        return True

    def line_of_sight(self, p: tuple[float, float], q: tuple[float, float]) -> bool:
        n = max(2, int(math.hypot(q[0] - p[0], q[1] - p[1]) / 0.05))
        for k in range(1, n):
            x, y = p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n
            if self._cell(x, y) in self.sight_blockers:
                return False
        return True

    def ray_distance(self, p: tuple[float, float], bearing_rad: float, max_d: float) -> float | None:
        """Distance to the first wall / obstacle along a ray, None if clear up to max_d."""
        d = 0.0
        while d < max_d:
            d += 0.05
            if self._cell(p[0] + d * math.cos(bearing_rad), p[1] + d * math.sin(bearing_rad)) in self.sight_blockers:
                return d
        return None

    # ---------------------------------------------------------------- planning
    def plan(self, start: tuple[float, float], goal: tuple[float, float], zones=()) -> list[tuple[float, float]] | None:
        """A* on the inflated grid (8-connected), then string-pulled. Returns waypoints (start excluded) or None.

        `zones` = circles (x, y, r) or rectangles (x0, y0, x1, y1) that must not be entered.
        """
        s, g = self._cell(*start), self._cell(*goal)
        zone_cells: set[tuple[int, int]] = set()
        for z in zones:
            zone_cells |= self._raster_rect(Rect(*z)) if len(z) == 4 else self._raster_disc(*z)
        bad = self.blocked_cells | zone_cells
        if g in bad or not self.bounds.contains(*goal):
            return None
        bx = (self._cell(self.bounds.x1, self.bounds.y1))

        def h(c):
            dx, dy = abs(c[0] - g[0]), abs(c[1] - g[1])
            return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

        openq = [(h(s), 0.0, s)]
        best = {s: 0.0}
        came: dict = {}
        while openq:
            _, cost, c = heapq.heappop(openq)
            if c == g:
                break
            if cost > best.get(c, 1e18):
                continue
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == dj == 0:
                        continue
                    n = (c[0] + di, c[1] + dj)
                    if n in bad or not (0 <= n[0] <= bx[0] and 0 <= n[1] <= bx[1]):
                        continue
                    nc = cost + (math.sqrt(2) if di and dj else 1.0)
                    if nc < best.get(n, 1e18):
                        best[n] = nc
                        came[n] = c
                        heapq.heappush(openq, (nc + h(n), nc, n))
        if g not in came and g != s:
            return None
        cells = [g]
        while cells[-1] != s:
            cells.append(came[cells[-1]])
        pts = [self._centre(c) for c in reversed(cells)]
        pts[0] = start
        pts[-1] = goal
        out, i = [], 0                                   # string pulling
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self.line_free(pts[i], pts[j], zones):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    @staticmethod
    def path_length(start: tuple[float, float], path: list[tuple[float, float]]) -> float:
        L, p = 0.0, start
        for q in path:
            L += math.hypot(q[0] - p[0], q[1] - p[1])
            p = q
        return L


def flat_lab() -> World:
    """Two-room lab (see scripts/gen_world_flat_lab.py): x -2..6.5, y -3..3, divider at x=2.5, doorway y 0..1."""
    t = 0.10
    X0, X1, Y0, Y1, DX = -2.0, 6.5, -3.0, 3.0, 2.5
    walls = [Rect(X0 - t / 2, Y0 - t / 2, X1 + t / 2, Y0 + t / 2), Rect(X0 - t / 2, Y1 - t / 2, X1 + t / 2, Y1 + t / 2),
             Rect(X0 - t / 2, Y0, X0 + t / 2, Y1), Rect(X1 - t / 2, Y0, X1 + t / 2, Y1),
             Rect(DX - t / 2, Y0, DX + t / 2, 0.0), Rect(DX - t / 2, 1.0, DX + t / 2, Y1)]
    w = World(
        bounds=Rect(X0 - 0.2, Y0 - 0.2, X1 + 0.2, Y1 + 0.2),
        rooms={"room A": Rect(X0, Y0, DX, Y1), "room B": Rect(DX, Y0, X1, Y1)},
        walls=walls,
        obstacles=[Rect(0.6, -1.55, 1.2, -1.25)],   # the table (not a detector class)
        objects=[GtObject(1, "box", "red", 1.0, 1.6, 0.17), GtObject(2, "box", "blue", 1.8, -2.2, 0.2),
                 GtObject(3, "barrel", "green", 4.5, 1.6, 0.2), GtObject(4, "box", "yellow", 4.6, -1.5, 0.2),
                 GtObject(5, "fire_extinguisher", "red", 5.8, -2.5, 0.1), GtObject(6, "mannequin", "grey", 3.6, 2.3, 0.25)],
        places={"home": (0.0, 0.0), "room A": (0.25, 0.0), "room B": (4.5, 0.0), "doorway": (2.5, 0.5)},
    )
    return w
