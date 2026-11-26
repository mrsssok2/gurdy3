"""Safety-shield test scenarios for the 2D kinematic simulator (used by tests and scripts/shield_sim_report.py).

Each scenario -> dict(walls, driver, pose, T, scan_ok); run once with and once without the shield.
Footprint = Nav2 footprint from robot.yaml (gurdy_navigation.footprint.footprint_polygon, margin 0.04), copied here so
the tests need neither numpy nor ROS; a ROS-image test checks it against robot.yaml.
"""
from __future__ import annotations

import math

from gurdy_core.shield_sim import constant, rect, waypoint_driver

FOOTPRINT = [[-0.273, -0.202], [0.0, -0.285], [0.273, -0.202], [0.273, 0.202], [0.0, 0.285], [-0.273, 0.202]]
LIMITS = (0.15, 0.10, 0.60)

# flat_lab divider: x = 2.5, thickness 0.1, doorway y in [0, 1] (scripts/gen_world_flat_lab.py)
X3 = 0.4125          # robot x at t = 3 s driving 0.15 m/s from rest (latency 0.05 s, lag 0.2 s; measured in the sim)

DOOR = rect(2.45, -3.0, 2.55, 0.0) + rect(2.45, 1.0, 2.55, 3.0)


def _sudden(t_appear, x0):
    base = rect(-3.0, -3.0, 9.0, -2.9) + rect(-3.0, 2.9, 9.0, 3.0)
    box = rect(x0, -0.4, x0 + 0.3, 0.4)
    return lambda t: base + (box if t >= t_appear else [])


def scenarios():
    s = {}
    s["wall_full_speed"] = dict(walls=rect(1.5, -2.0, 1.6, 2.0), driver=constant((0.15, 0.0, 0.0)), T=15.0,
                                note="teleop full speed (0.15 m/s) straight at a wall 1.23 m ahead")
    s["wall_reverse_strafe"] = dict(walls=rect(-2.0, -1.0, 2.0, -0.9), driver=constant((0.0, -0.10, 0.0)), T=12.0,
                                    note="full-speed strafe right at a wall 0.62 m to the side")
    s["corner_diagonal"] = dict(walls=rect(1.2, -2.0, 1.3, 1.1) + rect(-2.0, 1.0, 1.3, 1.1),
                                driver=constant((0.15, 0.10, 0.0)), T=20.0,
                                note="diagonal (vx 0.15, vy 0.10) into a corner")
    s["turn_near_wall"] = dict(walls=rect(-2.0, 0.40, 2.0, 0.50), driver=constant((0.0, 0.0, 0.6)), T=10.0,
                               note="turn in place at 0.6 rad/s, side 0.115 m from a wall (corners sweep to 0.34 m)")
    s["turn_tight"] = dict(walls=rect(-2.0, 0.33, 2.0, 0.43), driver=constant((0.0, 0.0, 0.6)), T=10.0,
                           note="turn in place at 0.6 rad/s, side 0.045 m from a wall: corners (0.34 m) would hit it")
    s["drive_turn_along_wall"] = dict(walls=rect(-2.0, 0.45, 6.0, 0.55), driver=constant((0.15, 0.0, 0.3)), T=10.0,
                                      note="vx 0.15 + wz 0.3 (left turn) with a wall 0.165 m to the left")
    s["doorway_centre"] = dict(walls=DOOR, driver=waypoint_driver([(4.0, 0.5)], LIMITS), pose=(1.0, 0.5, 0.0), T=40.0,
                               note="flat_lab doorway (1.0 m), straight through the centre, full speed")
    s["doorway_offset"] = dict(walls=DOOR, driver=waypoint_driver([(4.0, 0.35)], LIMITS), pose=(1.0, 0.35, 0.0), T=40.0,
                               note="same doorway, path 0.15 m off-centre (0.065 m from the jamb, inside d_safe)")
    s["doorway_diagonal"] = dict(walls=DOOR, driver=waypoint_driver([(2.0, 0.5), (3.0, 0.5), (4.0, -1.0)], LIMITS),
                                 pose=(1.0, -1.0, 0.0), T=60.0, note="approach the doorway at an angle, turn through it")
    s["sudden_obstacle"] = dict(walls=_sudden(3.0, X3 + 0.273 + 0.50), driver=constant((0.15, 0.0, 0.0)), T=12.0,
                                note="box appears at t=3 s, 0.50 m ahead of the front, robot at 0.15 m/s")
    s["sudden_obstacle_close"] = dict(walls=_sudden(3.0, X3 + 0.273 + 0.05), driver=constant((0.15, 0.0, 0.0)), T=12.0,
                                      note="box appears at t=3 s only 0.05 m ahead of the front (inside d_safe)")
    s["stale_scan"] = dict(walls=rect(4.0, -2.0, 4.1, 2.0), driver=constant((0.15, 0.0, 0.0)), T=8.0,
                           scan_ok=lambda t: t < 2.0, note="lidar stops publishing at t=2 s while driving")
    s["no_scan"] = dict(walls=rect(4.0, -2.0, 4.1, 2.0), driver=constant((0.15, 0.0, 0.0)), T=5.0,
                        scan_ok=lambda t: False, note="no scan ever arrives")
    for v in s.values():
        v.setdefault("pose", (0.0, 0.0, 0.0))
        v.setdefault("scan_ok", lambda t: True)
    return s


def shield_config(**kw):
    from gurdy_core.safety_shield import ShieldConfig
    return ShieldConfig(footprint=FOOTPRINT, limits=LIMITS, **kw)


def run_scenario(name, shielded=True, latency=0.05, tau=0.2, noise=0.0, seed=0, **cfg):
    from gurdy_core.safety_shield import SafetyShield
    from gurdy_core.shield_sim import run
    sc = scenarios()[name]
    sh = SafetyShield(shield_config(**cfg)) if shielded else None
    return run(FOOTPRINT, sc["walls"], sc["driver"], shield=sh, T=sc["T"], pose=sc["pose"], latency=latency, tau=tau,
               noise=noise, seed=seed, scan_ok=sc["scan_ok"])


def doorway_pass_time(res, x_line=2.5):
    """Time at which the robot centre crosses the divider (None if never)."""
    for t, x, *_ in res.trace:
        if x >= x_line:
            return t
    return None


def distance_after(res, t0):
    """Path length driven after time t0 (from the trace)."""
    pts = [(x, y) for t, x, y, *_ in res.trace if t >= t0]
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
