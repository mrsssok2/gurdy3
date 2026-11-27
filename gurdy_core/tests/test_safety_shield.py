"""Safety shield (CBF) unit tests + 2D kinematic simulator scenarios. Stdlib only (no numpy, no ROS, no Gazebo)."""
import math
import random

import pytest

from gurdy_core.safety_shield import (SafetyShield, build_constraints, footprint_distance, scan_to_points, solve)
from gurdy_core.shield_scenarios import (FOOTPRINT, distance_after, doorway_pass_time, run_scenario, scenarios,
                                         shield_config)

POLY = shield_config().footprint
FRONT = 0.273          # footprint front edge x


def fresh(points, **kw):
    s = SafetyShield(shield_config(**kw))
    s.update_scan(points, 0.0)
    return s


def wall_x(x, y0=-1.0, y1=1.0, n=41):
    return [(x, y0 + (y1 - y0) * i / (n - 1)) for i in range(n)]


def wall_y(y, x0=-1.0, x1=1.0, n=41):
    return [(x0 + (x1 - x0) * i / (n - 1), y) for i in range(n)]


# ---------------------------------------------------------------- geometry
def test_footprint_distance_sign_and_normal():
    d, n = footprint_distance(POLY, (1.0, 0.0))
    assert d == pytest.approx(1.0 - FRONT) and n == pytest.approx((1.0, 0.0))
    d, n = footprint_distance(POLY, (0.2, 0.0))                   # inside, nearest edge = front
    assert d == pytest.approx(-(FRONT - 0.2)) and n == pytest.approx((1.0, 0.0))
    d, n = footprint_distance(POLY, (0.0, -1.0))
    assert d == pytest.approx(1.0 - 0.285) and n == pytest.approx((0.0, -1.0))


def test_scan_to_points_drops_invalid_returns():
    r = [1.0, math.inf, math.nan, 0.05, 9.0, 2.0]
    pts = scan_to_points(r, 0.0, math.pi / 2, range_min=0.15, range_max=8.0)
    assert len(pts) == 2
    assert pts[0] == pytest.approx((1.0, 0.0))
    assert pts[1] == pytest.approx((2.0 * math.cos(5 * math.pi / 2), 2.0 * math.sin(5 * math.pi / 2)))


def test_footprint_matches_robot_yaml():
    pytest.importorskip("numpy")
    try:
        from gurdy_navigation.footprint import footprint_polygon
    except ImportError:
        pytest.skip("gurdy_navigation not importable here")
    from gurdy_core.robot import load_robot
    assert footprint_polygon(load_robot()) == FOOTPRINT


# ---------------------------------------------------------------- the filter
def test_free_space_passes_unchanged():
    s = fresh(wall_x(5.0))
    u, st = s.filter((0.15, 0.05, 0.3), 0.1)
    assert u == pytest.approx((0.15, 0.05, 0.3)) and st.state == "pass" and st.reduction == 0.0


def test_approach_speed_limited_to_alpha_times_barrier():
    h = 0.05
    s = fresh([(FRONT + 0.10 + h, 0.0)], alpha=2.0, d_safe=0.10)
    u, st = s.filter((0.15, 0.0, 0.0), 0.0)
    assert u[0] == pytest.approx(2.0 * h, abs=1e-5) and st.state == "active"
    assert st.reduction == pytest.approx((0.15 - 0.1) / 0.15, abs=1e-4)
    assert st.min_clearance == pytest.approx(0.15)


def test_moving_away_and_sliding_along_a_wall_are_untouched():
    s = fresh(wall_x(FRONT + 0.11))                    # just outside d_safe
    for cmd in [(-0.15, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, -0.1, 0.0)]:
        u, st = s.filter(cmd, 0.0)
        assert u == pytest.approx(cmd, abs=1e-6), cmd


def test_inside_dsafe_blocks_approach_allows_retreat_and_never_moves_by_itself():
    s = fresh(wall_x(FRONT + 0.05))                    # 0.05 < d_safe
    assert s.filter((0.0, 0.0, 0.0), 0.0)[0] == (0.0, 0.0, 0.0)
    u, _ = s.filter((0.15, 0.0, 0.0), 0.0)
    assert abs(u[0]) < 1e-6 and abs(u[1]) < 1e-6
    u, _ = s.filter((-0.1, 0.0, 0.0), 0.0)
    assert u == pytest.approx((-0.1, 0.0, 0.0))


def test_self_hits_inside_or_on_the_footprint_are_ignored():
    pts = [(0.1, 0.0), (FRONT + 0.01, 0.0), (0.0, 0.29)]          # inside / 1 cm / 5 mm from the footprint
    s = fresh(pts)
    u, st = s.filter((0.15, 0.1, 0.6), 0.0)
    assert u == pytest.approx((0.15, 0.1, 0.6)) and st.n_self_hits == 3 and st.n_constraints == 0


def test_rotation_is_limited_when_corners_swing_into_a_wall():
    s = fresh(wall_y(0.33, -1.5, 1.5, 121))            # side 0.045 m from the wall, corners at 0.34 m
    u, st = s.filter((0.0, 0.0, 0.6), 0.0)
    assert st.state == "active"
    assert abs(u[2]) < 0.6 * 0.5 or u[1] < -1e-3       # the turn is cut, or it slides away from the wall to allow it


def test_no_scan_and_stale_scan_give_zero_then_recover():
    s = SafetyShield(shield_config(scan_timeout=0.5))
    u, st = s.filter((0.15, 0.0, 0.0), 0.0)
    assert u == (0.0, 0.0, 0.0) and st.state == "no_scan" and st.reduction == 1.0
    s.update_scan(wall_x(5.0), 1.0)
    assert s.filter((0.15, 0.0, 0.0), 1.4)[0] == pytest.approx((0.15, 0.0, 0.0))
    u, st = s.filter((0.15, 0.0, 0.0), 1.6)
    assert u == (0.0, 0.0, 0.0) and st.state == "stale"
    s.update_scan(wall_x(5.0), 1.7)
    assert s.filter((0.15, 0.0, 0.0), 1.7)[0] == pytest.approx((0.15, 0.0, 0.0))


def test_output_never_exceeds_the_limits():
    s = fresh(wall_x(5.0))
    u, _ = s.filter((1.0, -1.0, 5.0), 0.0)
    assert u == pytest.approx((0.15, -0.10, 0.60))


def test_scale_only_never_adds_or_reverses_an_axis():
    s = fresh(wall_y(0.33, -1.5, 1.5, 121), scale_only=True)
    u, _ = s.filter((0.0, 0.0, 0.6), 0.0)
    assert u[0] == 0.0 and u[1] == 0.0 and 0.0 <= u[2] < 0.6


def test_random_scans_output_is_feasible_minimal_and_bounded():
    rng = random.Random(1)
    cfg = shield_config()
    for _ in range(300):
        pts = []
        for _ in range(rng.randint(0, 40)):
            a, r = rng.uniform(-math.pi, math.pi), rng.uniform(0.2, 1.2)
            pts.append((r * math.cos(a), r * math.sin(a)))
        cons, _, _ = build_constraints(pts, cfg)
        u0 = (rng.uniform(-0.15, 0.15), rng.uniform(-0.1, 0.1), rng.uniform(-0.6, 0.6))
        u = solve(u0, cons, cfg)
        for c in cons:
            assert sum(a * v for a, v in zip(c.a, u)) >= c.b - 1e-6
        assert all(abs(u[i]) <= cfg.limits[i] + 1e-9 for i in range(3))
        if all(sum(a * v for a, v in zip(c.a, u0)) >= c.b for c in cons):
            assert u == pytest.approx(u0)


# ---------------------------------------------------------------- 2D simulator
COLLIDES_WITHOUT = {"wall_full_speed", "wall_reverse_strafe", "corner_diagonal", "turn_tight", "drive_turn_along_wall",
                    "sudden_obstacle", "sudden_obstacle_close"}


@pytest.mark.parametrize("name", sorted(scenarios()))
def test_scenario_no_collision_with_shield(name):
    r = run_scenario(name, shielded=True)
    assert not r.collided, name
    assert r.min_clearance > 0.0
    if name in COLLIDES_WITHOUT:
        assert run_scenario(name, shielded=False).collided, f"{name}: scenario too easy, collides without the shield"


@pytest.mark.parametrize("name", ["wall_full_speed", "corner_diagonal", "sudden_obstacle"])
def test_scenario_robust_to_noise_and_slower_robot(name):
    for seed in range(3):
        r = run_scenario(name, shielded=True, noise=0.01, seed=seed, latency=0.1, tau=0.4)
        assert not r.collided, (name, seed)


def test_doorway_not_slowed_down_much():
    for name in ("doorway_centre", "doorway_offset", "doorway_diagonal"):
        a, b = run_scenario(name, shielded=False), run_scenario(name, shielded=True)
        assert b.reached, name
        ta, tb = doorway_pass_time(a), doorway_pass_time(b)
        assert tb <= ta * 1.05, (name, ta, tb)


def test_stale_scan_stops_within_timeout():
    r = run_scenario("stale_scan", shielded=True)
    t_zero = next(t for t, *_rest, cmd_out, state in r.trace if t > 2.0 and state == "stale")
    tmo = shield_config().scan_timeout                     # 0.6 s default since 2026-11-27 (measured /scan gaps)
    assert t_zero <= 2.0 + tmo + 0.05                       # last scan at 1.9 s + timeout + one control period
    assert distance_after(r, t_zero + 1.0) < 1e-3          # fully stopped 1 s later (lag tau 0.2 s)
    assert run_scenario("no_scan", shielded=True).path_len == 0.0
