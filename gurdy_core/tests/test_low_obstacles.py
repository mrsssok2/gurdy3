"""Phase 9c: low obstacles from the body depth camera, tested with synthetic depth images (no Gazebo, no ROS)."""
import math

import numpy as np
import pytest

from gurdy_core import low_obstacles as LO

CAM = {"pos": [0.15, 0.0, 0.02], "pitch_down": 0.17}          # robot.yaml depth_camera
W, H = 320, 240
K = LO.intrinsics(W, H, 1.047)
T = LO.T_base_optical(CAM)
FLOOR_Z = -0.11


def render(boxes=(), floor_tilt=(0.0, 0.0), noise=0.0, seed=0, T_bo=T, floor_z=FLOOR_Z):
    """Depth (optical z) of a floor plane (base_link z = floor_z, tilted by roll / pitch about the point under the body
    centre: the body tilts, the floor stays) + axis-aligned boxes (centre, size) in base_link."""
    vs, us = np.mgrid[0:H, 0:W]
    fx, fy, cx, cy = K
    rays = np.stack([(us - cx) / fx, (vs - cy) / fy, np.ones_like(us, float)], -1).reshape(-1, 3)
    d = rays @ T_bo[:3, :3].T
    o = T_bo[:3, 3]
    n = LO.rpy_to_R(floor_tilt[0], floor_tilt[1], 0.0) @ np.array([0, 0, 1.0])
    p0 = np.array([0, 0, floor_z])
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((p0 - o) @ n) / (d @ n)
    t = np.where(t > 0, t, np.inf)
    for c, s in boxes:
        lo, hi = np.asarray(c) - np.asarray(s) / 2, np.asarray(c) + np.asarray(s) / 2
        with np.errstate(divide="ignore", invalid="ignore"):
            t1, t2 = (lo - o) / d, (hi - o) / d
        tmin, tmax = np.nanmax(np.minimum(t1, t2), 1), np.nanmin(np.maximum(t1, t2), 1)
        hit = (tmax >= tmin) & (tmin > 0)
        t = np.where(hit & (tmin < t), tmin, t)
    z = t.copy()                                                    # ray z (optical) = 1 -> t is the depth value
    if noise:
        fin = np.isfinite(z)
        z[fin] += np.random.default_rng(seed).normal(0, 1, fin.sum()) * noise * z[fin] ** 2
    z[~np.isfinite(z) | (z < 0.10) | (z > 5.0)] = np.nan
    return z.reshape(H, W).astype(np.float32)


CUBE = ((0.80, 0.0, FLOOR_Z + 0.025), (0.05, 0.05, 0.05))


def test_floor_only_gives_no_obstacles():
    r = LO.extract(render(), K, T)
    assert r.plane_fitted and len(r.obstacles) == 0 and r.n_floor > 1000
    assert r.plane[0][2] == pytest.approx(1.0, abs=1e-6) and -r.plane[1] == pytest.approx(FLOOR_Z, abs=1e-4)
    assert len(r.clearing) > 100


@pytest.mark.parametrize("tilt", [(0.0, 0.05), (0.04, -0.05), (-0.05, 0.03)])
def test_tilted_body_floor_fit_no_false_obstacles(tilt):
    """Walking pitch / roll of ~3 deg: with a fixed z threshold the far floor is up to 1.8 m x 0.05 rad = 9 cm off: it
    is either marked as an obstacle (floor higher) or hides real low obstacles (floor lower). The fitted plane is right."""
    r = LO.extract(render(floor_tilt=tilt), K, T)
    assert r.plane_fitted and len(r.obstacles) == 0
    fixed = LO.apply_T(T, LO.depth_to_points(render(floor_tilt=tilt), K, LO.LowObstacleParams())[0])
    assert np.sum(np.abs(fixed[:, 2] - FLOOR_Z) > 0.025) > 50     # floor points a fixed threshold gets wrong


def test_noise_floor_only_few_false_points():
    for s in range(3):
        r = LO.extract(render(noise=0.003, seed=s), K, T)
        assert len(r.xy) <= 3, len(r.xy)                         # a handful of 2 cm cells at most


def test_small_cube_detected_with_position_and_height():
    r = LO.extract(render([CUBE]), K, T)
    assert len(r.obstacles) > 20
    c, s = CUBE
    assert np.all(np.abs(r.obstacles[:, 0] - c[0]) <= s[0] / 2 + 0.01)
    assert np.all(np.abs(r.obstacles[:, 1] - c[1]) <= s[1] / 2 + 0.01)
    assert r.heights.max() == pytest.approx(0.05, abs=0.006)
    xs = [p[0] for p in r.xy]
    assert min(xs) == pytest.approx(c[0] - s[0] / 2, abs=0.02)   # nearest face seen


def test_small_cube_with_noise_and_tilt():
    r = LO.extract(render([CUBE], floor_tilt=(0.0, 0.04), noise=0.003, seed=1), K, T)
    near = [p for p in r.xy if abs(p[0] - 0.8) < 0.05 and abs(p[1]) < 0.05]
    assert len(near) >= 3 and len(r.xy) - len(near) <= 3


def test_own_leg_in_view_is_filtered():
    leg_c, leg_s = (0.30, 0.05, -0.05), (0.02, 0.02, 0.10)         # raised front tibia in front of the lens
    d = render([leg_s and (leg_c, leg_s)])
    r_no = LO.extract(d, K, T)
    assert len(r_no.obstacles) > 10                                  # without the self filter it is an obstacle
    cap = ((0.30, 0.05, -0.10), (0.30, 0.05, 0.0), 0.015)
    r = LO.extract(d, K, T, self_capsules=[cap])
    assert len(r.obstacles) == 0 and r.n_self > 10


def test_leg_capsules_from_robot_model():
    pytest.importorskip("yaml")
    from gurdy_core.robot import load_robot
    caps = LO.leg_capsules(load_robot())
    assert len(caps) == 12
    feet = [c[1] for c in caps[1::2]]
    assert all(f[2] == pytest.approx(FLOOR_Z + 0.015, abs=1e-6) for f in feet)       # foot centre = radius above floor
    pts = np.array([f + [0, 0, 0.0] for f in feet])
    assert LO.capsule_mask(pts, caps).all()


def test_floor_level_sheet_is_not_an_obstacle_but_a_low_box_is():
    sheet = ((0.7, 0.0, FLOOR_Z + 0.005), (0.2, 0.2, 0.01))        # 1 cm: below min_h
    box = ((0.7, 0.0, FLOOR_Z + 0.02), (0.2, 0.2, 0.04))            # 4 cm: above min_h
    assert len(LO.extract(render([sheet]), K, T).obstacles) == 0
    assert len(LO.extract(render([box]), K, T).obstacles) > 20


def test_max_range():
    far = ((2.5, 0.0, FLOOR_Z + 0.05), (0.1, 0.4, 0.1))
    assert len(LO.extract(render([far]), K, T).obstacles) == 0
    assert len(LO.extract(render([far]), K, T, LO.LowObstacleParams(max_range=3.0)).obstacles) > 0


def test_shield_input_format_and_downsample():
    r = LO.extract(render([CUBE]), K, T)
    pts = LO.shield_points(r, memory_xy=[(0.5, 0.5)])
    assert all(isinstance(p, tuple) and len(p) == 2 for p in pts) and pts[-1] == (0.5, 0.5)
    cells = {(math.floor(x / 0.02), math.floor(y / 0.02)) for x, y in r.xy}
    assert len(cells) == len(r.xy) < len(r.obstacles)


def test_shield_accepts_points_and_slows_toward_the_cube():
    """The low-obstacle points work as SafetyShield input (Phase 8b, branch phase8b-shield)."""
    ss = pytest.importorskip("gurdy_core.safety_shield")
    from gurdy_core.shield_scenarios import shield_config
    r = LO.extract(render([((0.42, 0.0, FLOOR_Z + 0.025), (0.05, 0.05, 0.05))]), K, T)
    sh = ss.SafetyShield(shield_config())
    sh.update_scan(LO.shield_points(r), 0.0)
    u, st = sh.filter((0.15, 0.0, 0.0), 0.0)
    assert st.state == "active" and u[0] < 0.15


def test_memory_keeps_points_out_of_view_and_clears_when_seen_through():
    mem = LO.LowObstacleMemory(ttl=10.0)
    T_odom_base = np.eye(4)
    r = LO.extract(render([CUBE]), K, T)
    mem.add(LO.apply_T(T_odom_base, r.obstacles), 0.0)
    n0 = len(mem.points())
    assert n0 > 0
    # robot turned 90 deg: the cube is out of view -> nothing cleared, points kept
    T_turn = LO.make_T(LO.rpy_to_R(0, 0, math.pi / 2), [0, 0, 0])
    d_turn = render()
    assert mem.clear_seen_through(d_turn, K, T_turn @ T, LO.LowObstacleParams()) == 0 and len(mem.points()) == n0
    # cube removed, robot looks at the spot again: the floor behind is seen -> forgotten
    assert mem.clear_seen_through(render(), K, T_odom_base @ T, LO.LowObstacleParams()) > 0.8 * n0
    # time to live
    mem.add(LO.apply_T(T_odom_base, r.obstacles), 1.0)
    mem.prune(5.0)
    assert len(mem.points()) > 0
    mem.prune(12.0)
    assert len(mem.points()) == 0
