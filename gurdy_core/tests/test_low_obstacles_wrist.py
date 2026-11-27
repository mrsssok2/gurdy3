"""Phase 9f: the wrist RGB-D as a second low-obstacle source (synthetic wrist frames from tests/servo_sim.py)."""
import math

import numpy as np
import pytest

from gurdy_core import arm as A
from gurdy_core import low_obstacles as LO
from gurdy_core.robot import load_robot

from servo_sim import SimRobot

ROBOT = load_robot()
ARM = A.load_arm()


def frame(r):
    d, K, _ = r.wrist_view()
    return d, K


def world_at(r, rel):
    c, s = math.cos(r.yaw), math.sin(r.yaw)
    return r.x + c * rel[0] - s * rel[1], r.y + s * rel[0] + c * rel[1]


def test_floor_only_nothing():
    r = SimRobot("small_box", (5.0, 5.0))                 # object far away, out of view
    res = LO.extract_wrist(*frame(r), ROBOT, ARM, r.q)
    assert res.plane_fitted and len(res.xy) == 0


def test_object_behind_the_robot_is_seen():
    """Backing up to an object: the body camera cannot see it, the wrist camera (servo_view) does."""
    r = SimRobot("small_box", (-0.36, 0.05))
    res = LO.extract_wrist(*frame(r), ROBOT, ARM, r.q)
    assert len(res.obstacles) > 10
    c = np.mean(res.obstacles[:, :2], axis=0)
    assert np.hypot(*(c - np.array([-0.36, 0.05]))) < 0.03
    assert res.heights.max() == pytest.approx(0.05, abs=0.01)


def test_held_object_blinds_part_of_the_view_but_makes_no_obstacle():
    """The grasp centre is only 4.5 cm in front of the wrist lens (inside the 0.07 m near clip), so a held object is hit
    by every ray inside its silhouette before 0.07 m: it is a HOLE in the depth image, never a false obstacle. The
    blinded share of the image is measured (documented: the wrist camera sees less while carrying)."""
    g = A.fk(ARM, ARM.poses["servo_view"])["Gc"]

    def held(robot):
        x, y = world_at(robot, g[:2])
        return dict(kind="cylinder", centre=(x, y, robot.base_z + g[2] - 0.06), radius=0.033, height=0.12)
    r0 = SimRobot("small_box", (5.0, 5.0))
    r = SimRobot("small_box", (5.0, 5.0), extra=[held])
    d0, _ = frame(r0)
    d, K = frame(r)
    blind = float(np.mean(np.isnan(d)) - np.mean(np.isnan(d0)))
    assert blind > 0.2                                   # a held can hides a large part of the wrist view
    assert len(LO.extract_wrist(d, K, ROBOT, ARM, r.q, held=(0.033, 0.12)).obstacles) == 0


def test_held_object_capsule_covers_the_object_volume():
    q = ARM.poses["servo_view"]
    g = A.fk(ARM, q)["Gc"]
    caps = LO.wrist_self_capsules(ARM, q, opening=0.07, held_radius=0.033, held_height=0.12)
    ang = np.linspace(0, 2 * np.pi, 12, endpoint=False)
    pts = np.array([g + [0.033 * np.cos(t), 0.033 * np.sin(t), -z] for t in ang for z in (0.0, 0.05, 0.11)])
    assert LO.capsule_mask(pts, caps).all()
    far = np.array([g + [0.12, 0.0, -0.06]])
    assert not LO.capsule_mask(far, caps).any()


def test_own_leg_in_wrist_view_is_filtered():
    caps = LO.leg_capsules(ROBOT)
    foot = caps[5][1]                                     # lr tibia end (rear left foot), base_link

    def leg(robot):
        x, y = world_at(robot, foot[:2])
        return dict(kind="cylinder", centre=(x, y, 0.025), radius=0.012, height=0.05)
    r = SimRobot("small_box", (5.0, 5.0), extra=[leg])
    d, K = frame(r)
    raw = LO.extract(d, K, A.T_base_wrist_optical(ARM, r.q), LO.wrist_params())
    if len(raw.obstacles) == 0:
        pytest.skip("rear foot not in the wrist view from this pose")
    assert len(LO.extract_wrist(d, K, ROBOT, ARM, r.q).obstacles) == 0


def test_tilted_body_and_noise():
    for s, tilt in enumerate([(0.0, 0.04), (0.03, -0.03)]):
        r = SimRobot("can", (5.0, 5.0), tilt=tilt, depth_noise=0.003, seed=s)
        assert len(LO.extract_wrist(*frame(r), ROBOT, ARM, r.q).xy) <= 3


def test_merge_for_the_shield():
    r = SimRobot("small_box", (-0.36, 0.05))
    w = LO.extract_wrist(*frame(r), ROBOT, ARM, r.q)
    body = LO.LowObstacleResult(np.array([[0.8, 0.0, -0.09]]), np.array([0.02]), np.zeros((0, 3)), w.plane, True)
    m = LO.merge([w, body, None])
    assert len(m.obstacles) == len(w.obstacles) + 1
    assert (0.8, 0.0) in [(round(x, 2), round(y, 2)) for x, y in m.xy]
    assert len(LO.merge([]).obstacles) == 0
