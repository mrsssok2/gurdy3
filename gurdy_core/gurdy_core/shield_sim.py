"""Small 2D kinematic simulator for testing the safety shield without Gazebo (stdlib only).

World = wall segments (static, or a function of time for a sudden obstacle). Robot = holonomic body velocity (vx, vy, wz)
in its own frame, with a command LATENCY (s) and a first-order velocity LAG tau (s) so the robot does not obey instantly
(the RL policy needs a few strides to change speed). Lidar: 360 beams at 10 Hz from the body centre, optional Gaussian
noise, range [0.15, 8] m like robot.yaml. Control at 20 Hz, physics at 100 Hz. Clearance = exact distance between the
footprint polygon and the nearest wall segment (0 = touching / overlapping = collision).

This is a kinematic model: no contact forces, no slipping, no body sway; footprint = Nav2 footprint (feet hull).
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field


def rect(x0, y0, x1, y1):
    """Axis-aligned solid box -> its 4 edges."""
    a, b, c, d = (x0, y0), (x1, y0), (x1, y1), (x0, y1)
    return [(a, b), (b, c), (c, d), (d, a)]


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _seg_intersect(p1, p2, q1, q2):
    d1, d2, d3, d4 = _cross(q1, q2, p1), _cross(q1, q2, p2), _cross(p1, p2, q1), _cross(p1, p2, q2)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def _pt_seg(p, a, b):
    ex, ey = b[0] - a[0], b[1] - a[1]
    L2 = ex * ex + ey * ey
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / L2))
    return math.hypot(p[0] - a[0] - t * ex, p[1] - a[1] - t * ey)


def seg_seg_dist(p1, p2, q1, q2):
    if _seg_intersect(p1, p2, q1, q2):
        return 0.0
    return min(_pt_seg(p1, q1, q2), _pt_seg(p2, q1, q2), _pt_seg(q1, p1, p2), _pt_seg(q2, p1, p2))


def _inside(poly, p):
    return all(_cross(a, b, p) >= 0 for a, b in zip(poly, poly[1:] + poly[:1]))


def body_to_world(pose, pts):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in pts]


def clearance(poly_world, segs):
    """Distance between the (CCW convex) footprint and the walls; 0 if any wall touches or enters it."""
    best = math.inf
    edges = list(zip(poly_world, poly_world[1:] + poly_world[:1]))
    for q1, q2 in segs:
        if _inside(poly_world, q1) or _inside(poly_world, q2):
            return 0.0
        for p1, p2 in edges:
            best = min(best, seg_seg_dist(p1, p2, q1, q2))
            if best == 0.0:
                return 0.0
    return best


def raycast(pose, segs, n=360, rmin=0.15, rmax=8.0, noise=0.0, rng=None):
    """-> ranges (angle_min = -pi, increment 2pi/n, body frame), inf for no return."""
    x, y, th = pose
    out = []
    for i in range(n):
        a = th - math.pi + i * 2 * math.pi / n
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        for (ax, ay), (bx, by) in segs:
            ex, ey = bx - ax, by - ay
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((ax - x) * ey - (ay - y) * ex) / den
            u = ((ax - x) * dy - (ay - y) * dx) / den
            if t > 0 and -1e-9 <= u <= 1 + 1e-9 and t < best:
                best = t
        if math.isfinite(best) and noise and rng is not None:
            best += rng.gauss(0.0, noise)
        out.append(best if rmin <= best <= rmax else math.inf)
    return out


@dataclass
class SimResult:
    collided: bool
    min_clearance: float
    t_end: float
    reached: bool
    path_len: float
    trace: list = field(default_factory=list)      # (t, x, y, th, clearance, cmd_nom, cmd_out, state)


def run(footprint, walls, driver, shield=None, T=20.0, pose=(0.0, 0.0, 0.0), latency=0.05, tau=0.2,
        noise=0.0, seed=0, scan_ok=lambda t: True, dt=0.01, ctrl_dt=0.05, scan_dt=0.1, stop_on_collision=False):
    """walls: list of segments or callable t -> segments. driver(t, pose) -> (vx, vy, wz) or None (= goal reached).
    scan_ok(t) False = the lidar publishes nothing at t (stale / no scan tests)."""
    rng = random.Random(seed)
    wall_at = walls if callable(walls) else (lambda t, w=walls: w)
    x, y, th = pose
    v = [0.0, 0.0, 0.0]
    queue = []                                   # (t_apply, u)
    u_app = (0.0, 0.0, 0.0)
    min_c, collided, reached, path = math.inf, False, False, 0.0
    trace = []
    nom, out, state = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), "-"
    steps = int(round(T / dt))
    k_ctrl, k_scan = int(round(ctrl_dt / dt)), int(round(scan_dt / dt))
    t = 0.0
    for k in range(steps + 1):
        t = k * dt
        segs = wall_at(t)
        if shield is not None and k % k_scan == 0 and scan_ok(t):
            r = raycast((x, y, th), segs, noise=noise, rng=rng)
            shield.update_laserscan(r, -math.pi, 2 * math.pi / len(r), 0.15, 8.0, t)
        if k % k_ctrl == 0:
            cmd = driver(t, (x, y, th))
            if cmd is None:
                reached = True
                cmd = (0.0, 0.0, 0.0)
            nom = tuple(cmd)
            if shield is not None:
                out, st = shield.filter(nom, t)
                state = st.state
            else:
                out = nom
            queue.append((t + latency, out))
        while queue and queue[0][0] <= t + 1e-9:
            u_app = queue.pop(0)[1]
        c = clearance(body_to_world((x, y, th), footprint), segs)
        min_c = min(min_c, c)
        if c <= 0.0:
            collided = True
        if k % k_ctrl == 0:
            trace.append((round(t, 3), x, y, th, c, nom, out, state))
        if reached or (collided and stop_on_collision):
            break
        for i in range(3):
            v[i] += (u_app[i] - v[i]) * min(1.0, dt / tau)
        cs, sn = math.cos(th), math.sin(th)
        dx, dy = (cs * v[0] - sn * v[1]) * dt, (sn * v[0] + cs * v[1]) * dt
        x, y, th = x + dx, y + dy, th + v[2] * dt
        path += math.hypot(dx, dy)
    return SimResult(collided, min_c, t, reached, path, trace)


def constant(u, until=math.inf):
    return lambda t, pose: tuple(u) if t < until else (0.0, 0.0, 0.0)


def waypoint_driver(points, limits=(0.15, 0.10, 0.60), tol=0.05, k_head=1.5, face_motion=True):
    """Simple Nav2-like follower: full speed toward the next waypoint (holonomic, scaled into the limits), heading turned
    toward the motion direction. -> None after the last waypoint."""
    pts = list(points)
    idx = [0]

    def drive(t, pose):
        x, y, th = pose
        while idx[0] < len(pts):
            gx, gy = pts[idx[0]]
            if math.hypot(gx - x, gy - y) > tol:
                break
            idx[0] += 1
        if idx[0] >= len(pts):
            return None
        gx, gy = pts[idx[0]]
        ex, ey = gx - x, gy - y
        c, s = math.cos(-th), math.sin(-th)
        bx, by = c * ex - s * ey, s * ex + c * ey          # error in the body frame
        d = math.hypot(bx, by)
        sc = 1.0 / max(math.hypot(bx / d / limits[0], by / d / limits[1]), 1e-9)   # speed along the direction at the limit ellipse
        sp = min(sc, d * 1.5 + 0.03) if idx[0] == len(pts) - 1 else sc
        vx, vy = sp * bx / d, sp * by / d
        wz = 0.0
        if face_motion:
            err = math.atan2(by, bx)
            wz = max(-limits[2], min(limits[2], k_head * err))
        return (vx, vy, wz)
    return drive
