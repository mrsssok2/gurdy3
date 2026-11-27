"""Phase 8c: box + depth -> 3D object position (map frame), with synthetic depth images (no Gazebo, no ROS)."""
import math

import numpy as np
import pytest

from gurdy_core.vision.object3d import (Object3DParams, Track3D, compare_3d, depth_cluster, estimate)
from gurdy_core.vision.synth_depth import projected_box, render
from gurdy_core.vision.transforms import (R_to_quat, T_base_optical, T_from_tq, T_map_base, apply_T, compose,
                                          intrinsics_from_fov, inv_T, pixel_rays, project, quat_to_R, rpy_to_R)

CAM = {"pos": [0.15, 0.0, 0.02], "pitch_down": 0.17, "width": 320, "height": 240, "hfov": 1.047}   # robot.yaml
W, H = 320, 240
K = intrinsics_from_fov(W, H, CAM["hfov"])
BODY_Z = 0.11                                        # base_link height when standing (nominal_stance.body_height)


def cam_T(x=0.0, y=0.0, yaw=0.0, **kw):
    return T_map_base(x, y, yaw, z=BODY_Z, **kw) @ T_base_optical(CAM)


def xy_err(o, c):
    return math.hypot(o.centre[0] - c[0], o.centre[1] - c[1])


def one(solid, T, others=(), noise=0.0, seed=0, **kw):
    sol = [solid, *others]
    d, _, boxes = render(K, W, H, T, sol, noise=noise, rng=np.random.default_rng(seed))
    assert boxes[0] is not None, "object not in view"
    return estimate(d, boxes[0], K, T, **kw), boxes[0], d


CUBE = dict(kind="box", centre=(1.4, 0.1, 0.10), size=(0.20, 0.20, 0.20), yaw=0.0)


# ------------------------------------------------------------------ transforms
def test_rpy_quat_roundtrip_and_inverse():
    rng = np.random.default_rng(3)
    for _ in range(50):
        rpy = rng.uniform(-math.pi, math.pi, 3) * [0.5, 0.45, 1.0]
        R = rpy_to_R(*rpy)
        assert np.allclose(R @ R.T, np.eye(3)) and np.isclose(np.linalg.det(R), 1.0)
        assert np.allclose(quat_to_R(R_to_quat(R)), R, atol=1e-9)
        T = T_from_tq(rng.normal(size=3), R_to_quat(R))
        p = rng.normal(size=(5, 3))
        assert np.allclose(apply_T(inv_T(T), apply_T(T, p)), p)
        assert np.allclose(compose(T, inv_T(T)), np.eye(4))


def test_urdf_camera_chain_looks_forward_and_down():
    T = T_base_optical(CAM)
    z_axis, x_axis, y_axis = T[:3, 2], T[:3, 0], T[:3, 1]
    p = CAM["pitch_down"]
    assert np.allclose(z_axis, [math.cos(p), 0.0, -math.sin(p)], atol=1e-9)     # optical z = forward, pitched down
    assert np.allclose(x_axis, [0.0, -1.0, 0.0], atol=1e-9)                     # optical x = right of the robot
    assert y_axis[2] < 0                                                        # optical y = down
    assert np.allclose(T[:3, 3], CAM["pos"])


def test_intrinsics_and_projection_roundtrip():
    fx, fy, cx, cy = K
    assert fx == pytest.approx(160 / math.tan(1.047 / 2)) and fx == fy and (cx, cy) == (160.0, 120.0)
    pts = np.array([[0.1, -0.05, 1.0], [-0.3, 0.2, 2.5]])
    uv = project(K, pts)
    back = pixel_rays(K, uv[:, 0], uv[:, 1]) * pts[:, 2:3]
    assert np.allclose(back, pts)


def test_floor_pixels_backproject_to_z0_in_any_pose():
    for pose in [(0, 0, 0), (1.5, -2.0, 2.4)]:
        T = cam_T(*pose)
        d, ids, _ = render(K, W, H, T, [])
        vs, us = np.nonzero(np.isfinite(d))
        p = apply_T(T, pixel_rays(K, us, vs) * d[vs, us, None])
        assert len(p) > 1000 and np.abs(p[:, 2]).max() < 1e-6


# ------------------------------------------------------------------ single sighting
def test_cube_on_floor_centre_top_and_size():
    o, box, _ = one(CUBE, cam_T())
    assert o is not None and o.on_floor and o.top_known and o.width_known
    assert xy_err(o, CUBE["centre"]) < 0.015      # push-back along the ray, not the face normal: ~ radius * sin(4.8 deg)
    assert o.centre[2] == pytest.approx(0.10, abs=0.01)
    assert o.top_z == pytest.approx(0.20, abs=0.01) and o.height == pytest.approx(0.20, abs=0.01)
    assert o.width == pytest.approx(0.20, abs=0.02)


def test_cylinder_extinguisher_like():
    c = dict(kind="cylinder", centre=(2.0, -0.3, 0.25), radius=0.08, height=0.50)
    o, _, _ = one(c, cam_T())
    assert xy_err(o, c["centre"]) < 0.02 and o.top_z == pytest.approx(0.50, abs=0.02) and o.width == pytest.approx(0.16, abs=0.02)


def test_same_map_estimate_from_another_robot_pose():
    """The map-frame result must not depend on where the robot stands (TF chain applied correctly)."""
    c = CUBE["centre"]
    for x, y, yaw in [(0.0, 0.0, 0.0), (0.6, 0.9, -0.6), (2.6, 0.5, math.pi), (1.4, -1.0, math.pi / 2)]:
        o, _, _ = one(CUBE, cam_T(x, y, yaw))
        assert xy_err(o, c) < 0.015, (x, y, yaw)
        assert o.top_z == pytest.approx(0.20, abs=0.012)


def test_rotated_box_two_faces_visible():
    b = dict(kind="box", centre=(1.5, 0.0, 0.10), size=(0.25, 0.25, 0.20), yaw=0.6)
    o, _, _ = one(b, cam_T())
    assert xy_err(o, b["centre"]) < 0.03


def test_background_wall_behind_and_beside_is_rejected():
    wall = dict(kind="box", centre=(1.9, 0.0, 0.5), size=(0.05, 4.0, 1.0), yaw=0.0)        # 0.4 m behind the cube
    o, box, d = one(CUBE, cam_T(), others=[wall])
    pad = (box[0] - 15, box[1] - 15, box[2] + 15, box[3] + 15)                              # loose YOLO box
    o2 = estimate(d, pad, K, cam_T())
    for e in (o, o2):
        assert xy_err(e, CUBE["centre"]) < 0.02 and e.top_z == pytest.approx(0.20, abs=0.015)


def test_wall_only_3cm_behind_is_still_rejected():
    """The object's own depth (front face -> wall) is the jump that separates them, not the gap behind it."""
    wall = dict(kind="box", centre=(1.555, 0.0, 0.5), size=(0.05, 4.0, 1.0), yaw=0.0)      # wall face 3 cm behind the cube
    o, box, d = one(CUBE, cam_T(), others=[wall])
    pad = (box[0] - 15, box[1] - 15, box[2] + 15, box[3] + 15)
    for e in (o, estimate(d, pad, K, cam_T())):
        assert xy_err(e, CUBE["centre"]) < 0.02 and e.top_z == pytest.approx(0.20, abs=0.015)


def test_noise_keeps_error_small():
    errs = [xy_err(one(CUBE, cam_T(), noise=0.003, seed=s)[0], CUBE["centre"]) for s in range(5)]
    assert max(errs) < 0.02


def test_truncated_top_is_flagged_as_lower_bound():
    tall = dict(kind="cylinder", centre=(1.2, 0.0, 0.45), radius=0.275, height=0.90)       # barrel: top out of view
    o, box, _ = one(tall, cam_T())
    assert not o.top_known and o.top_z < 0.90
    assert xy_err(o, tall["centre"]) < 0.03


def test_side_truncated_object_flags_width():
    o, box, _ = one(dict(CUBE, centre=(0.9, 0.55, 0.10)), cam_T())
    assert box[2] <= W and (o is None or (o.truncated["left"] or o.truncated["right"] or o.width_known))


def test_floor_only_or_empty_box_returns_none():
    T = cam_T()
    d, _, _ = render(K, W, H, T, [])
    assert estimate(d, (100, 150, 200, 230), K, T) is None                       # only floor in the box
    assert estimate(np.full((H, W), np.nan, np.float32), (100, 100, 200, 200), K, T) is None
    assert estimate(d, (10, 10, 10, 10), K, T) is None                            # empty box


def test_depth_cluster_splits_at_gap():
    z = np.array([1.00, 1.01, 1.02, 1.03, 1.50, 1.51, 2.0])
    m = depth_cluster(z, gap=0.06, seed_pct=5, max_extent=1.5)
    assert m.tolist() == [True, True, True, True, False, False, False]


def test_covariance_grows_with_range_and_is_along_the_bearing():
    near, _, _ = one(dict(CUBE, centre=(1.0, 0.0, 0.10)), cam_T())
    far, _, _ = one(dict(CUBE, centre=(3.5, 0.0, 0.10)), cam_T())
    assert np.trace(far.cov) > np.trace(near.cov)
    assert far.cov[0, 0] > far.cov[1, 1]                    # robot looks along +x: along-ray sigma > lateral


def test_elongated_object_end_on_single_view_bias_and_two_view_fusion():
    """Toolbox 0.45 x 0.20 seen end-on: one view cannot see its depth -> centre ~0.12 m short (even with the class radius
    prior 0.10). Seen broadside it is right (prior caps the radius). Fusing both views (information filter) halves the
    error. Documented limit of single-view estimates."""
    tb = dict(kind="box", centre=(2.0, 0.0, 0.10), size=(0.45, 0.20, 0.20), yaw=0.0)
    end_on, _, _ = one(tb, cam_T(0.0, 0.0, 0.0), radius_prior=0.10)
    assert 0.08 < xy_err(end_on, tb["centre"]) < 0.16
    side, _, _ = one(tb, cam_T(2.0, -1.5, math.pi / 2), radius_prior=0.10)
    assert xy_err(side, tb["centre"]) < 0.03
    t = Track3D()
    t.update(end_on)
    t.update(side)
    assert xy_err(type("O", (), {"centre": t.centre})(), tb["centre"]) < xy_err(end_on, tb["centre"]) / 2


# ------------------------------------------------------------------ fusion + evaluation
def test_track_fuses_noisy_sightings_and_rejects_an_outlier():
    t = Track3D()
    for s, (x, y, yaw) in enumerate([(0, 0, 0), (0.3, 0.4, -0.3), (0.2, -0.5, 0.3), (0.5, 0.0, 0.0), (0.0, 0.3, -0.2)]):
        o, _, _ = one(CUBE, cam_T(x, y, yaw), noise=0.003, seed=s)
        assert t.update(o)
    bad, _, _ = one(dict(CUBE, centre=(2.4, 0.1, 0.10)), cam_T())      # a different object 1 m further
    assert not t.update(bad) and t.rejected == 1
    s = t.summary()
    assert math.hypot(s["x"] - 1.4, s["y"] - 0.1) < 0.01 and s["top_z"] == pytest.approx(0.20, abs=0.01)
    assert s["hits"] == 5 and all(v < 0.02 for v in s["sigma"])


def test_compare_3d_against_ground_truth():
    gt = [{"name": "cube", "class": "box", "x": 1.4, "y": 0.1, "z": 0.10, "size": [0.2, 0.2, 0.2]},
          {"name": "ext", "class": "fire_extinguisher", "x": 2.0, "y": -0.3, "z": 0.25, "size": [0.16, 0.16, 0.5]}]
    found = [{"class": "box", "x": 1.41, "y": 0.1, "z": 0.11, "top_z": 0.21, "height": 0.21},
             {"class": "box", "x": 5.0, "y": 5.0, "z": 0.1}]
    r = compare_3d(found, gt)
    assert r["recall"] == 0.5 and r["false"] == 1
    row = r["rows"][0]
    assert row["xy_err"] == pytest.approx(0.01) and row["z_err"] == pytest.approx(0.01) and row["top_err"] == pytest.approx(0.01)
    assert r["xyz_err"]["n"] == 1


def test_projected_box_contains_visible_box():
    T = cam_T()
    _, _, boxes = render(K, W, H, T, [CUBE])
    pb = projected_box(K, W, H, T, CUBE)
    vb = boxes[0]
    assert pb[0] <= vb[0] + 1 and pb[1] <= vb[1] + 1 and pb[2] >= vb[2] - 1 and pb[3] >= vb[3] - 1


def test_params_stride_and_min_points():
    o, box, d = one(CUBE, cam_T())
    o1 = estimate(d, box, K, cam_T(), Object3DParams(stride=1))
    assert xy_err(o1, CUBE["centre"]) < 0.015 and o1.n_points > o.n_points
    assert estimate(d, box, K, cam_T(), Object3DParams(min_points=10 ** 6)) is None


# ------------------------------------------------------------------ class shape prior fits
@pytest.mark.parametrize("yaw", [0.0, 0.4, 0.785, 1.2, 2.5])
def test_rectangle_fit_any_yaw(yaw):
    b = dict(kind="box", centre=(1.8, 0.2, 0.175), size=(0.40, 0.40, 0.35), yaw=yaw)
    ray, _, _ = one(b, cam_T(), radius_prior=0.20)
    fit, _, _ = one(b, cam_T(), shape={"kind": "box", "size": (0.40, 0.40)})
    assert xy_err(fit, b["centre"]) < 0.015 and xy_err(fit, b["centre"]) <= xy_err(ray, b["centre"]) + 1e-3


def test_rectangle_fit_resolves_elongated_end_on():
    tb = dict(kind="box", centre=(2.0, 0.0, 0.10), size=(0.45, 0.20, 0.20), yaw=0.0)
    fit, _, _ = one(tb, cam_T(), shape={"kind": "box", "size": (0.45, 0.20)})
    assert xy_err(fit, tb["centre"]) < 0.02
    assert min(abs(fit.fit_yaw), abs(fit.fit_yaw - math.pi)) < math.radians(5)


def test_circle_fit_known_radius():
    c = dict(kind="cylinder", centre=(2.4, -0.4, 0.45), radius=0.275, height=0.90)
    fit, _, _ = one(c, cam_T(), noise=0.003, shape={"kind": "cylinder", "radius": 0.275})
    assert xy_err(fit, c["centre"]) < 0.01


def test_box_cut_short_by_the_detector_makes_top_unknown():
    tall = dict(kind="cylinder", centre=(2.0, 0.0, 0.25), radius=0.08, height=0.50)
    o, box, d = one(tall, cam_T())
    assert o.top_known
    cut = (box[0], box[1] + 0.4 * (box[3] - box[1]), box[2], box[3])          # detector box misses the upper 40 %
    o2 = estimate(d, cut, K, cam_T())
    assert not o2.top_known and o2.top_z < 0.45
