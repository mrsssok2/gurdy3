"""Nav2 footprint and velocity limits derived from robot.yaml (no ROS imports).

The footprint is the convex hull of the six nominal-stance feet, pushed outward by the foot radius
plus `margin` (stride overshoot and gait sway), and always containing the body box. Nav2 params are
patched with this at launch time, so changing robot.yaml changes the navigation geometry too.
"""
from __future__ import annotations

import numpy as np

from gurdy_core.kinematics import nominal_foot
from gurdy_core.robot import LEG_NAMES

# Nav2 minimum speeds (enforced by cmd_vel_min_speed_node) live in gurdy_core.rl_command.
from gurdy_core.rl_command import MIN_ANGULAR_SPEED as MIN_ANGULAR, MIN_LINEAR_SPEED as MIN_LINEAR  # noqa: E402,F401


def foot_points_body(robot) -> np.ndarray:
    """(6, 2) nominal-stance foot centres in the body frame."""
    reach = nominal_foot(robot)[0]
    pts = []
    for leg in LEG_NAMES:
        m = robot.mounts[leg]
        pts.append([m.xy[0] + reach * np.cos(m.yaw), m.xy[1] + reach * np.sin(m.yaw)])
    return np.array(pts)


def cross2(a, b) -> float:
    """z of the 2-D cross product a x b (np.cross on 2-vectors was removed in numpy 2)."""
    return float(a[0] * b[1] - a[1] * b[0])


def _hull(points: np.ndarray) -> np.ndarray:
    """Convex hull (Andrew monotone chain), counter-clockwise."""
    pts = sorted(map(tuple, np.round(points, 6)))

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and cross2(np.subtract(out[-1], out[-2]), np.subtract(p, out[-2])) <= 0:
                out.pop()
            out.append(p)
        return out

    lower, upper = half(pts), half(reversed(pts))
    return np.array(lower[:-1] + upper[:-1])


def footprint_polygon(robot, margin: float = 0.04) -> list[list[float]]:
    feet = foot_points_body(robot)
    grow = robot.foot_radius + margin
    pts = []
    for p in feet:
        n = np.linalg.norm(p)
        pts.append(p + grow * p / n)
    sx, sy = robot.body_size[0] / 2, robot.body_size[1] / 2
    pts += [[sx, sy], [sx, -sy], [-sx, sy], [-sx, -sy]]
    return [[round(float(x), 3) + 0.0, round(float(y), 3) + 0.0] for x, y in _hull(np.array(pts))]


def max_extent(poly) -> float:
    """Largest distance from the body centre to a footprint vertex (turning-in-place radius)."""
    return float(max(np.hypot(x, y) for x, y in poly))


def velocity_limits(robot) -> dict:
    c = robot.command_limits
    return {"vx": float(c["vx"]), "vy": float(c["vy"]), "wz": float(c["wz"])}


def patch_nav2_params(params: dict, robot, margin: float = 0.04, odom_topic: str | None = None) -> dict:
    """Return `params` (parsed nav2_params.yaml) with footprint + velocity limits from robot.yaml.
    odom_topic: odometry topic Nav2 reads (default /odom; /odometry/filtered with the EKF)."""
    import copy
    p = copy.deepcopy(params)
    poly = footprint_polygon(robot, margin)
    text = "[" + ", ".join(f"[{x}, {y}]" for x, y in poly) + "]"
    for costmap in ("local_costmap", "global_costmap"):
        p[costmap][costmap]["ros__parameters"]["footprint"] = text
    v = velocity_limits(robot)
    mppi = p["controller_server"]["ros__parameters"]["FollowPath"]
    mppi["vx_max"], mppi["vx_min"] = v["vx"], -v["vx"]
    mppi["vy_max"] = v["vy"]
    mppi["wz_max"] = v["wz"]
    sm = p["velocity_smoother"]["ros__parameters"]
    sm["max_velocity"] = [v["vx"], v["vy"], v["wz"]]
    sm["min_velocity"] = [-v["vx"], -v["vy"], -v["wz"]]
    if odom_topic:
        p["bt_navigator"]["ros__parameters"]["odom_topic"] = odom_topic
        sm["odom_topic"] = odom_topic
        # MPPI takes the robot's current velocity from the controller_server's own odometry subscription. Left at the default
        # /odom (which does not exist with the EKF) MPPI believes the robot never moves: it re-starts from v = 0 every cycle,
        # so its commands stay at ax_max * dt (~0.04 m/s, mostly below the min-speed threshold) and the robot stalls.
        p["controller_server"]["ros__parameters"]["odom_topic"] = odom_topic
    return p


def beam_exit_radius(poly, angles, origin=(0.0, 0.0), margin: float = 0.0) -> np.ndarray:
    """For each beam angle (rad, in the sensor frame, sensor at `origin` inside the body frame), the distance at which a ray
    from the sensor leaves the (convex, sensor-containing) footprint polygon grown by `margin` metres. Returns on a beam
    closer than this are the robot itself (swinging femurs reach the lidar plane) or a collision, never a useful obstacle."""
    pts = np.array(poly, dtype=float) - np.asarray(origin, dtype=float)
    if margin:
        n = np.linalg.norm(pts, axis=1, keepdims=True)
        pts = pts + margin * pts / n
    a = np.asarray(angles, dtype=float)
    d = np.stack([np.cos(a), np.sin(a)], axis=1)
    out = np.full(len(a), np.inf)
    for p, q in zip(pts, np.roll(pts, -1, axis=0)):
        e = q - p
        denom = d[:, 0] * e[1] - d[:, 1] * e[0]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (p[0] * e[1] - p[1] * e[0]) / denom          # ray parameter
            u = (p[0] * d[:, 1] - p[1] * d[:, 0]) / denom    # segment parameter
        hit = (np.abs(denom) > 1e-12) & (t > 0) & (u >= -1e-9) & (u <= 1 + 1e-9)
        out = np.where(hit, np.minimum(out, t), out)
    return out


def filter_self_hits(ranges, exit_radius) -> np.ndarray:
    """Copy of `ranges` with every finite return inside the footprint replaced by +inf (= no return)."""
    r = np.asarray(ranges, dtype=float).copy()
    r[np.isfinite(r) & (r < np.asarray(exit_radius))] = np.inf
    return r
