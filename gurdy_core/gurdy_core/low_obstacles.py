"""Phase 9c: low obstacles (below the LiDAR plane) from the body RGB-D camera. numpy only, no ROS.

    depth image (optical z, m) --K--> optical points --T_base_opt--> base_link points
      -> range / validity filter -> RANSAC floor plane (per frame: the body pitches / rolls a few degrees while walking,
         so a fixed z threshold would mark the floor far away) -> height above the floor
      -> self filter (leg capsules from the joint angles, optional arm capsules, the body box)
      -> obstacle points (height in [min_h, max_h]) + clearing points (floor, for costmap ray tracing)
      -> 2D grid downsample -> shield input: [(x, y), ...] in base_link (gurdy_core.safety_shield.SafetyShield.update_scan)
    LowObstacleMemory keeps obstacle points in the odom frame for `ttl` s (the camera sees only 60 deg ahead and nothing
    closer than ~0.35 m in front of base_link) and forgets a point when the camera looks THROUGH it (measured depth at its
    pixel clearly farther).
Camera geometry = robot.yaml sensors.depth_camera and the URDF chain (base_link -> camera_link pitch_down -> optical).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


# ------------------------------------------------------------------ geometry helpers (self-contained)
def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def make_T(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def apply_T(T, p):
    p = np.asarray(p, float)
    return p @ T[:3, :3].T + T[:3, 3]


def T_base_optical(cam: dict) -> np.ndarray:
    """robot.yaml depth_camera (pos, pitch_down) -> T_base_link_camera_optical_frame (gurdy.urdf.xacro chain)."""
    T_bc = make_T(rpy_to_R(0.0, float(cam.get("pitch_down", 0.0)), 0.0), cam["pos"])
    return T_bc @ make_T(rpy_to_R(-math.pi / 2, 0.0, -math.pi / 2), [0, 0, 0])


def intrinsics(width, height, hfov):
    f = (width / 2) / math.tan(hfov / 2)
    return f, f, width / 2, height / 2


# ------------------------------------------------------------------ parameters / result
@dataclass
class LowObstacleParams:
    stride: int = 2                  # pixel step (320 x 240 -> 160 x 120 = 19 200 points max)
    min_range: float = 0.15          # m (optical z); camera clip is 0.10
    max_range: float = 1.8           # m: depth noise and tiny objects beyond (a 5 cm cube is ~8 px at 1.8 m)
    min_h: float = 0.025             # m above the fitted floor = obstacle (floor noise / plane error below this)
    max_h: float = 0.45              # m (higher things: the LiDAR / costmap already see most; arm-height clearance)
    floor_prior_z: float = -0.11     # floor z in base_link standing (nominal_stance.body_height)
    floor_band: float = 0.06         # RANSAC candidates: points within this of the prior plane
    floor_tol: float = 0.012         # RANSAC inlier distance
    floor_max_tilt: float = 0.17     # rad: reject fitted planes tilted more than this from the prior (10 deg)
    ransac_iters: int = 60
    min_floor_points: int = 200
    self_margin: float = 0.02        # m added to every self capsule / the body box
    grid: float = 0.02               # m: 2D downsample cell for the shield / costmap
    clear_stride: int = 4            # extra decimation of the clearing (floor) cloud
    seed: int = 0


@dataclass
class LowObstacleResult:
    obstacles: np.ndarray            # (N, 3) base_link, height above floor in [min_h, max_h], self points removed
    heights: np.ndarray              # (N,) height above the fitted floor
    clearing: np.ndarray             # (M, 3) base_link floor points (decimated): free space for costmap ray tracing
    plane: tuple                     # (n (3,), d): n . p + d = 0, n up, in base_link
    plane_fitted: bool               # False: prior plane used (too few floor points / fit rejected)
    n_valid: int = 0
    n_self: int = 0
    n_floor: int = 0
    xy: list = field(default_factory=list)   # grid-downsampled obstacle (x, y) in base_link = shield input


# ------------------------------------------------------------------ steps
def depth_to_points(depth, K, prm: LowObstacleParams):
    """-> (optical points (N, 3), pixel (N, 2) u, v) of valid depth pixels in [min_range, max_range]."""
    fx, fy, cx, cy = K
    H, W = depth.shape[:2]
    vs, us = np.mgrid[0:H:prm.stride, 0:W:prm.stride]
    z = depth[vs, us].astype(float)
    ok = np.isfinite(z) & (z >= prm.min_range) & (z <= prm.max_range)
    u, v, z = us[ok].astype(float), vs[ok].astype(float), z[ok]
    return np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], axis=1), np.stack([u, v], axis=1)


def fit_floor(pts, prm: LowObstacleParams):
    """RANSAC plane among points near the prior floor (z = floor_prior_z). -> (n, d, fitted, inlier mask)."""
    prior = (np.array([0.0, 0.0, 1.0]), -prm.floor_prior_z)
    cand = np.abs(pts[:, 2] - prm.floor_prior_z) <= prm.floor_band
    P = pts[cand]
    if len(P) < prm.min_floor_points:
        return prior[0], prior[1], False, np.abs(pts @ prior[0] + prior[1]) <= prm.floor_tol
    rng = np.random.default_rng(prm.seed)
    best_n, best_d, best_cnt = None, None, -1
    for _ in range(prm.ransac_iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        if n[2] < 0:
            n = -n
        if math.acos(min(1.0, n[2])) > prm.floor_max_tilt:
            continue
        d = -float(n @ a)
        cnt = int(np.sum(np.abs(P @ n + d) <= prm.floor_tol))
        if cnt > best_cnt:
            best_n, best_d, best_cnt = n, d, cnt
    if best_n is None or best_cnt < prm.min_floor_points:
        return prior[0], prior[1], False, np.abs(pts @ prior[0] + prior[1]) <= prm.floor_tol
    inl = np.abs(P @ best_n + best_d) <= prm.floor_tol                 # least-squares refit on the inliers
    Q = P[inl]
    c = Q.mean(axis=0)
    _, _, vt = np.linalg.svd(Q - c, full_matrices=False)
    n = vt[-1] if vt[-1][2] > 0 else -vt[-1]
    d = -float(n @ c)
    return n, d, True, np.abs(pts @ n + d) <= prm.floor_tol


def capsule_mask(pts, capsules) -> np.ndarray:
    """True for points inside any capsule (p0, p1, radius)."""
    m = np.zeros(len(pts), bool)
    for p0, p1, r in capsules:
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        d = p1 - p0
        L2 = float(d @ d)
        t = np.clip(((pts - p0) @ d) / L2, 0, 1) if L2 > 1e-12 else np.zeros(len(pts))
        m |= np.linalg.norm(pts - (p0 + t[:, None] * d), axis=1) <= r
    return m


def box_mask(pts, half, margin) -> np.ndarray:
    return np.all(np.abs(pts) <= np.asarray(half) + margin, axis=1)


def leg_capsules(robot, q: dict | None = None, radius: float = 0.02) -> list:
    """Coxa+femur and tibia capsules of all six legs in base_link for joint angles q {joint name: rad}
    (default: nominal stance). Uses gurdy_core.kinematics (same FK as the gait / RL)."""
    from .kinematics import nominal_joint_angles
    from .robot import LEG_NAMES
    qn = nominal_joint_angles(robot)
    if q:
        qn.update({k: float(v) for k, v in q.items() if k in qn})
    l1, l2, l3 = robot.lengths
    caps = []
    for leg in LEG_NAMES:
        m = robot.mounts[leg]
        yaw, pitch, knee = qn[f"{leg}_hip_yaw"], qn[f"{leg}_hip_pitch"], qn[f"{leg}_knee"]
        c, s = math.cos(m.yaw), math.sin(m.yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        hip = np.array([m.xy[0], m.xy[1], 0.0])
        u = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        knee_p = (l1 + l2 * math.cos(pitch)) * u + np.array([0, 0, l2 * math.sin(pitch)])
        foot_p = knee_p + l3 * (math.cos(pitch - knee) * u + np.array([0, 0, math.sin(pitch - knee)]))
        caps.append((hip, hip + R @ knee_p, radius + 0.005))
        caps.append((hip + R @ knee_p, hip + R @ foot_p, radius))
    return caps


def grid_xy(pts, cell) -> list:
    """Unique 2D cells (cell centre of the lowest-index point per cell) -> [(x, y), ...]."""
    if len(pts) == 0:
        return []
    keys = np.floor(pts[:, :2] / cell).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return [(float(x), float(y)) for x, y in pts[np.sort(idx), :2]]


def extract(depth, K, T_base_opt, prm: LowObstacleParams | None = None, self_capsules=(), body_half=(0.15, 0.09, 0.03)):
    """One depth frame -> LowObstacleResult (all in base_link)."""
    prm = prm or LowObstacleParams()
    p_opt, _ = depth_to_points(depth, K, prm)
    pts = apply_T(T_base_opt, p_opt) if len(p_opt) else np.zeros((0, 3))
    n_valid = len(pts)
    self_m = (capsule_mask(pts, [(a, b, r + prm.self_margin) for a, b, r in self_capsules]) | box_mask(pts, body_half, prm.self_margin)) \
        if len(pts) else np.zeros(0, bool)
    pts_ns = pts[~self_m]
    n, d, fitted, floor_m = fit_floor(pts_ns, prm) if len(pts_ns) else (np.array([0, 0, 1.0]), -prm.floor_prior_z, False, np.zeros(0, bool))
    h = pts_ns @ n + d if len(pts_ns) else np.zeros(0)
    obst = (h >= prm.min_h) & (h <= prm.max_h)
    clearing = pts_ns[floor_m][::prm.clear_stride]
    res = LowObstacleResult(pts_ns[obst], h[obst], clearing, (n, d), fitted, n_valid, int(self_m.sum()), int(floor_m.sum()))
    res.xy = grid_xy(res.obstacles, prm.grid)
    return res


def shield_points(res: LowObstacleResult, memory_xy=()) -> list:
    """Shield input (base_link (x, y) list): current low obstacles + remembered ones (already in base_link)."""
    return list(res.xy) + [tuple(map(float, p)) for p in memory_xy]


# ------------------------------------------------------------------ short-term memory (odom frame)
class LowObstacleMemory:
    """Low-obstacle points in the odom frame (grid-deduplicated) with a time to live. Needed because the camera sees
    only ahead: an obstacle the robot walks up to (< 0.35 m in front) or turns away from would vanish from the input."""

    def __init__(self, ttl: float = 10.0, cell: float = 0.02, clear_margin: float = 0.05):
        self.ttl, self.cell, self.clear_margin = ttl, cell, clear_margin
        self.cells = {}                    # (i, j) -> (x, y, z, t)

    def add(self, pts_odom, t):
        for x, y, z in np.asarray(pts_odom, float).reshape(-1, 3):
            self.cells[(int(math.floor(x / self.cell)), int(math.floor(y / self.cell)))] = (x, y, z, float(t))

    def prune(self, t):
        self.cells = {k: v for k, v in self.cells.items() if t - v[3] <= self.ttl}

    def clear_seen_through(self, depth, K, T_odom_opt, prm: LowObstacleParams):
        """Forget points that project into the current image where the measured depth is clearly farther (the camera
        sees past them -> gone). Points outside the image / beyond max_range / without depth are kept."""
        if not self.cells:
            return 0
        fx, fy, cx, cy = K
        H, W = depth.shape[:2]
        T_opt_odom = np.linalg.inv(T_odom_opt)
        keys = list(self.cells)
        P = apply_T(T_opt_odom, np.array([self.cells[k][:3] for k in keys]))
        gone = 0
        for k, (x, y, z) in zip(keys, P):
            if z < prm.min_range or z > prm.max_range:
                continue
            u, v = int(round(fx * x / z + cx)), int(round(fy * y / z + cy))
            if 0 <= u < W and 0 <= v < H:
                m = depth[v, u]
                if np.isfinite(m) and m > z + self.clear_margin:
                    del self.cells[k]
                    gone += 1
        return gone

    def points(self) -> np.ndarray:
        return np.array([v[:3] for v in self.cells.values()]) if self.cells else np.zeros((0, 3))


# ------------------------------------------------------------------ Phase 9f: wrist RGB-D as a second source
def wrist_params() -> LowObstacleParams:
    """Wrist camera (arm.yaml): 0.07-1.0 m clip; looking down from <= 0.31 m, so a wider floor band for the RANSAC prior."""
    return LowObstacleParams(min_range=0.07, max_range=1.0, floor_band=0.08)


def wrist_self_capsules(arm, q_arm, opening: float = 0.08, held_radius: float | None = None, held_height: float = 0.0,
                        margin: float = 0.01) -> list:
    """Arm + gripper capsules (gurdy_core.arm.arm_capsules, finger opening from the joint states) and, while an object is
    held, a capsule around it: from the grasp centre down by held_height (a can hangs ~half its height below the
    fingers), radius = object radius. All in base_link."""
    from . import arm as A
    caps = [(p0, p1, r + margin) for _, p0, p1, r in A.arm_capsules(arm, q_arm, opening)]
    if held_radius is not None:
        g = A.fk(arm, q_arm)["Gc"]
        caps.append((g + np.array([0, 0, held_height / 2]), g - np.array([0, 0, held_height]), held_radius + margin))
    return caps


def extract_wrist(depth, K, robot, arm, q_arm, q_legs=None, opening: float = 0.08, held=None, prm: LowObstacleParams | None = None):
    """One wrist depth frame -> LowObstacleResult in base_link. held = (radius, height) of the object in the gripper."""
    from . import arm as A
    prm = prm or wrist_params()
    T = A.T_base_wrist_optical(arm, q_arm)
    caps = leg_capsules(robot, q_legs) + wrist_self_capsules(arm, q_arm, opening, *(held or (None, 0.0)))
    return extract(depth, K, T, prm, caps, tuple(np.array(robot.body_size) / 2))


def merge(results, cell: float = 0.02) -> LowObstacleResult:
    """Combine results of several cameras (all base_link) for the shield: obstacle points concatenated, grid again."""
    rs = [r for r in results if r is not None]
    if not rs:
        return LowObstacleResult(np.zeros((0, 3)), np.zeros(0), np.zeros((0, 3)), (np.array([0, 0, 1.0]), 0.11), False)
    out = LowObstacleResult(np.vstack([r.obstacles for r in rs]), np.concatenate([r.heights for r in rs]),
                            np.vstack([r.clearing for r in rs]), rs[0].plane, any(r.plane_fitted for r in rs),
                            sum(r.n_valid for r in rs), sum(r.n_self for r in rs), sum(r.n_floor for r in rs))
    out.xy = grid_xy(out.obstacles, cell)
    return out
