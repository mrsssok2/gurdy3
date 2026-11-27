"""LiDAR safety shield (control barrier function), no ROS, stdlib only.

Last filter before the robot: it takes the command (vx, vy, wz) from ANY source and returns the closest command (in the
command-limit-scaled norm) that keeps every LiDAR point at least `d_safe` metres outside the robot footprint.

Barrier per obstacle point p (body frame, fixed in the world):
    h(p)  = dist(p, footprint) - d_safe                    (dist < 0 inside the polygon)
    dh/dt = n . p_dot,  p_dot = (-vx + wz*py, -vy - wz*px)  (n = unit vector from the nearest footprint point to p)
    CBF condition  dh/dt >= -alpha * h   <=>   a . u >= b,  a = (-nx, -ny, nx*py - ny*px),  b = -alpha * h
Far from obstacles the condition is inactive (command passes unchanged); close to one only the velocity component TOWARD
it is reduced (sliding along a wall or through a doorway keeps its speed), and the approach speed shrinks to zero
smoothly as h -> 0. Rotation is constrained too: a footprint corner swinging into a wall has a lever-arm term in a.

QP  min ||S^-1 (u - u_nom)||^2  s.t.  A u >= b,  |u_i| <= limit_i  (S = diag(limits)) is solved with Dykstra's alternating
projections (exact projection onto an intersection of convex sets) and then verified; if numerical leftovers violate a
constraint, the result is scaled toward zero. Inside d_safe the barrier is capped at 'no further approach' (b <= 0), so
u = 0 is always feasible, a stop always passes unchanged and the shield never pushes the robot around on its own.

Fail safe: no scan yet, or a scan older than `scan_timeout` -> zero command.
Self-hits: points within `self_margin` of the footprint (or inside it) are the robot's own legs (lidar plane is inside the
femur swing) and are ignored, the same rule as scan_self_filter_node; a real obstacle that close is therefore invisible.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

ZERO = (0.0, 0.0, 0.0)


@dataclass
class ShieldConfig:
    footprint: list                          # convex polygon [[x, y], ...] in the body frame (robot.yaml via footprint_polygon)
    d_safe: float = 0.10                     # m kept between the footprint and any obstacle point
    alpha: float = 2.0                       # 1/s, CBF gain: allowed approach speed = alpha * (clearance - d_safe)
    limits: tuple = (0.15, 0.10, 0.60)       # |vx|, |vy|, |wz| the output never exceeds (robot.yaml command_limits)
    self_margin: float = 0.02                # points closer than this to the footprint = self-hits, ignored
    influence: float = 0.60                  # m beyond d_safe; points farther away never bind and are dropped
    sector_deg: float = 10.0                 # keep the nearest point per angular sector (bounds the QP size)
    scan_timeout: float = 0.6                # s; older scan -> zero command (0.6: day_0927 step 10, user 2026-11-27)
    lidar_origin: tuple = (0.0, 0.0)         # lidar position in the body frame (x, y)
    scale_only: bool = False                 # True: each axis may only shrink toward 0 (no added sideways / reverse motion)
    iters: int = 300                         # Dykstra iterations (max)
    tol: float = 1e-6
    sectors: int = field(init=False, default=0)

    def __post_init__(self):
        self.footprint = [tuple(map(float, p)) for p in self.footprint]
        if _area(self.footprint) < 0:
            self.footprint.reverse()                       # CCW
        self.sectors = max(1, int(round(360.0 / self.sector_deg)))


def _area(poly):
    return 0.5 * sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]))


def footprint_distance(poly, p):
    """-> (signed distance from p to the convex CCW polygon (negative inside), unit normal pointing from the polygon toward
    p (outward when inside))."""
    px, py = p
    best, bq = math.inf, None
    inside = True
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        ex, ey = x1 - x0, y1 - y0
        if ex * (py - y0) - ey * (px - x0) < 0:
            inside = False
        L2 = ex * ex + ey * ey
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x0) * ex + (py - y0) * ey) / L2))
        qx, qy = x0 + t * ex, y0 + t * ey
        d = math.hypot(px - qx, py - qy)
        if d < best:
            best, bq = d, (qx, qy)
    dx, dy = px - bq[0], py - bq[1]
    if best < 1e-12:                                     # on the boundary: use the direction from the centroid
        cx = sum(x for x, _ in poly) / len(poly)
        cy = sum(y for _, y in poly) / len(poly)
        dx, dy, best = px - cx, py - cy, 1.0
        n = math.hypot(dx, dy) or 1.0
        return 0.0, (dx / n, dy / n)
    n = (dx / best, dy / best)
    if inside:
        return -best, (-n[0], -n[1])
    return best, n


def scan_to_points(ranges, angle_min, angle_increment, range_min=0.0, range_max=math.inf, origin=(0.0, 0.0)):
    """LaserScan fields -> body-frame points (x, y). Invalid / out-of-range / inf / nan returns are dropped."""
    out = []
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r < range_min or r > range_max:
            continue
        a = angle_min + i * angle_increment
        out.append((origin[0] + r * math.cos(a), origin[1] + r * math.sin(a)))
    return out


@dataclass
class Constraint:
    a: tuple          # in physical units: a . u >= b
    b: float
    dist: float       # clearance of the point to the footprint (m)
    point: tuple


def build_constraints(points, cfg: ShieldConfig):
    """Nearest relevant point per angular sector -> CBF half-spaces. Also -> (min clearance seen, n self-hits dropped)."""
    best = {}
    n_self = 0
    for p in points:
        d, n = footprint_distance(cfg.footprint, p)
        if d < cfg.self_margin:
            n_self += 1
            continue
        if d > cfg.d_safe + cfg.influence:
            continue
        k = int(((math.atan2(p[1], p[0]) + math.pi) / (2 * math.pi)) * cfg.sectors) % cfg.sectors
        if k not in best or d < best[k][0]:
            best[k] = (d, n, p)
    cons = []
    for d, (nx, ny), (px, py) in best.values():
        # inside d_safe (h < 0) only 'no further approach' (b = 0): the shield never creates motion by itself, so u = 0 is
        # always feasible and a stop command always passes unchanged
        cons.append(Constraint((-nx, -ny, nx * py - ny * px), -cfg.alpha * max(d - cfg.d_safe, 0.0), d, (px, py)))
    min_d = min((c.dist for c in cons), default=math.inf)
    return cons, min_d, n_self


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def solve(u_nom, cons, cfg: ShieldConfig):
    """Minimal change of u_nom (scaled by the limits) satisfying every constraint and the box. Returns u."""
    L = cfg.limits
    s0 = [max(-1.0, min(1.0, u_nom[i] / L[i])) for i in range(3)]
    if cfg.scale_only:
        lo, hi = [min(0.0, v) for v in s0], [max(0.0, v) for v in s0]
    else:
        lo, hi = [-1.0] * 3, [1.0] * 3
    A = [tuple(c.a[i] * L[i] for i in range(3)) for c in cons]        # a.u = (a*L).s
    B = [c.b for c in cons]

    def ok(s, eps=1e-9):
        return all(_dot(a, s) >= b - eps for a, b in zip(A, B))

    if ok(s0):
        return tuple(s0[i] * L[i] for i in range(3))
    x = list(s0)
    P = [[0.0, 0.0, 0.0] for _ in range(len(A) + 1)]
    for _ in range(cfg.iters):
        prev = list(x)
        for j, (a, b) in enumerate(zip(A, B)):
            y = [x[i] + P[j][i] for i in range(3)]
            g = _dot(a, y)
            n2 = _dot(a, a)
            if g < b and n2 > 0:
                k = (b - g) / n2
                z = [y[i] + k * a[i] for i in range(3)]
            else:
                z = y
            P[j] = [y[i] - z[i] for i in range(3)]
            x = z
        y = [x[i] + P[-1][i] for i in range(3)]
        z = [max(lo[i], min(hi[i], y[i])) for i in range(3)]
        P[-1] = [y[i] - z[i] for i in range(3)]
        x = z
        if max(abs(x[i] - prev[i]) for i in range(3)) < cfg.tol and ok(x, 1e-6):
            break
    if not ok(x, 1e-6):
        # numerical leftovers: largest t in [0, 1] with t*x feasible (u = 0 is always feasible, every b <= 0)
        lo, hi = 0.0, 1.0
        for _ in range(40):
            m = 0.5 * (lo + hi)
            lo, hi = (m, hi) if ok([m * v for v in x]) else (lo, m)
        x = [lo * v for v in x]
    return tuple(x[i] * L[i] for i in range(3))


@dataclass
class ShieldStatus:
    state: str              # pass | active | stale | no_scan
    reduction: float        # ||S^-1 (u_nom - u)|| / ||S^-1 u_nom||  (0 = unchanged, 1 = fully removed)
    min_clearance: float    # m, nearest non-self point to the footprint (inf if none in range)
    n_constraints: int
    n_self_hits: int
    scan_age: float

    def as_dict(self):
        f = lambda v: None if not math.isfinite(v) else round(v, 4)   # noqa: E731
        return {"state": self.state, "active": self.state != "pass", "reduction": round(self.reduction, 4),
                "min_clearance": f(self.min_clearance), "n_constraints": self.n_constraints,
                "n_self_hits": self.n_self_hits, "scan_age": f(self.scan_age)}


def _reduction(u_nom, u, L):
    n0 = math.sqrt(sum((u_nom[i] / L[i]) ** 2 for i in range(3)))
    if n0 < 1e-12:
        return 0.0
    return min(1.0, math.sqrt(sum(((u_nom[i] - u[i]) / L[i]) ** 2 for i in range(3))) / n0)


class SafetyShield:
    """Holds the latest scan; `filter(u, now)` -> (u_safe, ShieldStatus). `now` and scan times share one clock (s)."""

    def __init__(self, cfg: ShieldConfig):
        self.cfg = cfg
        self.cons, self.min_d, self.n_self = [], math.inf, 0
        self.scan_time = None

    def update_scan(self, points, t):
        self.cons, self.min_d, self.n_self = build_constraints(points, self.cfg)
        self.scan_time = float(t)

    def update_laserscan(self, ranges, angle_min, angle_increment, range_min, range_max, t):
        self.update_scan(scan_to_points(ranges, angle_min, angle_increment, range_min, range_max, self.cfg.lidar_origin), t)

    def filter(self, u_nom, now):
        u_nom = tuple(float(v) for v in u_nom)
        if self.scan_time is None:
            return ZERO, ShieldStatus("no_scan", _reduction(u_nom, ZERO, self.cfg.limits), math.inf, 0, 0, math.inf)
        age = float(now) - self.scan_time
        if age > self.cfg.scan_timeout:
            return ZERO, ShieldStatus("stale", _reduction(u_nom, ZERO, self.cfg.limits), self.min_d, 0, self.n_self, age)
        if u_nom == ZERO:
            return ZERO, ShieldStatus("pass", 0.0, self.min_d, len(self.cons), self.n_self, age)
        u = solve(u_nom, self.cons, self.cfg)
        red = _reduction(u_nom, u, self.cfg.limits)
        return u, ShieldStatus("active" if red > 1e-6 else "pass", red, self.min_d, len(self.cons), self.n_self, age)
