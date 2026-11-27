"""Phase 8c wiring: locate3d (fitted floor plane in the map frame, class shape priors) + SemanticMap 3D tracks.
Synthetic depth images (gurdy_core.vision.synth_depth), no ROS, no Gazebo.

Frames as in Gazebo with ground-truth odometry: the renderer's world has the floor at z 0; the map / odom frame has
base_link at z 0 when standing, so the map floor is at z = -0.11 (measured, CLAUDE.md 8b/8c note)."""
import json
import math

import numpy as np
import pytest

from gurdy_core.vision import locate3d as L
from gurdy_core.vision.object3d import Object3DParams, estimate
from gurdy_core.vision.semantic_map import SemanticMap
from gurdy_core.vision.synth_depth import render
from gurdy_core.vision.transforms import T_base_optical, T_map_base, intrinsics_from_fov, make_T, rpy_to_R

CAM = {"pos": [0.15, 0.0, 0.02], "pitch_down": 0.17}
W, H = 320, 240
K = intrinsics_from_fov(W, H, 1.047)
BODY_Z = 0.11
FLOOR_MAP = -0.11
T_MAP_WORLD = make_T(t=(0.0, 0.0, FLOOR_MAP))        # map z = world z - 0.11


def world_cam(x=0.0, y=0.0, yaw=0.0, **kw):
    return T_map_base(x, y, yaw, z=BODY_Z, **kw) @ T_base_optical(CAM)


def shot(solids, x=0.0, y=0.0, yaw=0.0, noise=0.0, seed=0, **kw):
    """-> depth, boxes, T_map_opt (map frame, floor at -0.11)."""
    Tw = world_cam(x, y, yaw, **kw)
    d, _, boxes = render(K, W, H, Tw, solids, noise=noise, rng=np.random.default_rng(seed))
    return d, boxes, T_MAP_WORLD @ Tw


def gt_map(c):
    return np.array([c[0], c[1], c[2] + FLOOR_MAP])


CUBE = dict(kind="box", centre=(1.3, 0.1, 0.15), size=(0.40, 0.40, 0.30), yaw=0.6)      # box class, top visible at 1.3 m
# (yaw 0.3 seen from the origin is the grazing-side case: 3.7 cm, see docs/results/phase8c_plan.md section 8)
EXT = dict(kind="cylinder", centre=(1.5, -0.2, 0.25), radius=0.08, height=0.50)
TOOLBOX = dict(kind="box", centre=(1.2, 0.0, 0.10), size=(0.45, 0.20, 0.20), yaw=0.6)


# ------------------------------------------------------------------ shape priors
def test_shape_priors_per_class():
    assert L.shape_prior("box") == {"kind": "box", "size": (0.40, 0.40), "size_range": (0.25, 0.50)}
    assert L.shape_prior("toolbox") == {"kind": "box", "size": (0.45, 0.20)}
    assert L.shape_prior("barrel")["kind"] == "cylinder" and L.shape_prior("barrel")["radius"] == pytest.approx(0.275)
    assert L.shape_prior("fire_extinguisher")["radius"] == pytest.approx(0.08)
    assert L.shape_prior("can")["kind"] == "cylinder" and L.shape_prior("small_box")["kind"] == "box"
    for c in ("traffic_cone", "chair", "mannequin", "mini_cone", "unknown_thing"):
        assert L.shape_prior(c) is None
    assert L.radius_of("unknown_thing") is None
    assert L.yaw_period("box") == pytest.approx(math.pi / 2) and L.yaw_period("toolbox") == pytest.approx(math.pi)


def test_yaw_mean_is_circular_modulo_the_period():
    assert L.yaw_mean([0.01, math.pi - 0.01], math.pi) == pytest.approx(0.0, abs=1e-6) or \
        L.yaw_mean([0.01, math.pi - 0.01], math.pi) == pytest.approx(math.pi, abs=1e-6)
    assert L.yaw_mean([0.5, 0.6, 0.7], math.pi) == pytest.approx(0.6, abs=1e-6)
    assert L.yaw_mean([0.1, 0.1 + math.pi / 2], math.pi / 2) == pytest.approx(0.1, abs=1e-6)
    assert L.yaw_mean([], math.pi) is None


# ------------------------------------------------------------------ floor plane
def test_floor_fit_finds_the_map_floor_at_minus_011():
    d, _, T = shot([CUBE])
    ft = L.FloorTracker(L.FloorParams(floor_z=FLOOR_MAP))
    n, dd = ft.update(d, K, T, 0.0)
    assert ft.source == "fit" and ft.n_fit == 1
    assert L.plane_z_at((n, dd), 1.0, 0.0) == pytest.approx(FLOOR_MAP, abs=0.003)
    assert n[2] == pytest.approx(1.0, abs=1e-3)


def test_floor_fit_corrects_a_wrong_prior_and_rejects_a_far_one():
    d, _, T = shot([CUBE])
    ft = L.FloorTracker(L.FloorParams(floor_z=-0.09))            # 2 cm off: inside band and max_offset -> fitted -0.11
    assert L.plane_z_at(ft.update(d, K, T, 0.0), 1.0, 0.0) == pytest.approx(FLOOR_MAP, abs=0.003)
    ft = L.FloorTracker(L.FloorParams(floor_z=0.0))              # 11 cm off: no candidates -> prior plane
    assert L.plane_z_at(ft.update(d, K, T, 0.0), 1.0, 0.0) == pytest.approx(0.0)
    assert ft.source == "prior"


def test_floor_tracker_holds_the_last_fit_then_falls_back_to_the_prior():
    d, _, T = shot([CUBE])
    ft = L.FloorTracker(L.FloorParams(floor_z=-0.10, hold_s=2.0))
    ft.update(d, K, T, 0.0)
    blind = np.full_like(d, np.nan)                               # no floor in view (e.g. a wall right in front)
    assert L.plane_z_at(ft.update(blind, K, T, 1.0), 1, 0) == pytest.approx(FLOOR_MAP, abs=0.003) and ft.source == "held"
    assert L.plane_z_at(ft.update(blind, K, T, 5.0), 1, 0) == pytest.approx(-0.10) and ft.source == "prior"
    off = L.FloorTracker(L.FloorParams(floor_z=-0.10, fit=False))
    assert L.plane_z_at(off.update(d, K, T, 0.0), 1, 0) == pytest.approx(-0.10) and off.source == "prior"


# ------------------------------------------------------------------ locate
def test_locate_box_with_the_fitted_floor_gets_centre_height_and_yaw():
    d, boxes, T = shot([CUBE])
    plane = L.FloorTracker(L.FloorParams()).update(d, K, T, 0.0)
    s = L.locate(d, boxes[0], K, T, "box", plane)
    g = gt_map(CUBE["centre"])
    assert s.method == "fit" and s.obj.on_floor and s.top_known
    assert math.hypot(*(s.centre[:2] - g[:2])) < 0.01
    assert s.centre[2] == pytest.approx(g[2], abs=0.01)
    assert s.bottom_z == pytest.approx(FLOOR_MAP, abs=0.005) and s.floor_z == pytest.approx(FLOOR_MAP, abs=0.005)
    assert s.top_z == pytest.approx(FLOOR_MAP + 0.30, abs=0.01) and s.height == pytest.approx(0.30, abs=0.01)
    assert (s.yaw - CUBE["yaw"]) % (math.pi / 2) == pytest.approx(0.0, abs=0.04) or \
        (CUBE["yaw"] - s.yaw) % (math.pi / 2) == pytest.approx(0.0, abs=0.04)
    sd = s.to_dict()
    assert sd["method"] == "fit" and sd["top_known"] and len(sd["c"]) == 3 and len(sd["sigma"]) == 3
    json.dumps(sd)


def test_the_floor_matters_object3d_with_floor_z_0_in_the_map_frame_is_wrong():
    """Without the floor plane (object3d default floor_z 0) the lower 13 cm of the box count as 'floor' in the map frame."""
    d, boxes, T = shot([CUBE])
    naive = estimate(d, boxes[0], K, T, Object3DParams(), shape=L.shape_prior("box"))
    assert naive.height < 0.20 and abs(naive.centre[2] - gt_map(CUBE["centre"])[2]) > 0.04     # 0.19 m tall, centre 5 cm high
    s = L.locate(d, boxes[0], K, T, "box", (np.array([0.0, 0.0, 1.0]), -FLOOR_MAP))     # prior plane, no fit
    assert s.height == pytest.approx(0.30, abs=0.01)


def test_locate_corrects_a_small_camera_tilt_error_by_the_fitted_floor():
    """TF has a 1.5 deg pitch error (body sway at the image stamp): the fitted plane absorbs it, the prior plane does not."""
    Tw_true = world_cam(0, 0, 0)
    d, _, boxes = render(K, W, H, Tw_true, [CUBE])
    Tw_err = world_cam(0, 0, 0, pitch=math.radians(1.5))
    T_err = T_MAP_WORLD @ Tw_err
    fit = L.FloorTracker(L.FloorParams()).update(d, K, T_err, 0.0)
    assert L.FloorTracker(L.FloorParams()).source == "prior"           # fresh tracker default
    s_fit = L.locate(d, boxes[0], K, T_err, "box", fit)
    s_pri = L.locate(d, boxes[0], K, T_err, "box", (np.array([0.0, 0.0, 1.0]), -FLOOR_MAP))
    assert abs(s_fit.height - 0.30) < 0.015
    assert abs(s_fit.height - 0.30) < abs(s_pri.height - 0.30) or abs(s_pri.top_z - s_fit.top_z) > 0.01


def test_locate_cylinder_and_ray_classes():
    d, boxes, T = shot([EXT], noise=0.003, seed=2)
    plane = L.FloorTracker(L.FloorParams()).update(d, K, T, 0.0)
    s = L.locate(d, boxes[0], K, T, "fire_extinguisher", plane)
    g = gt_map(EXT["centre"])
    assert s.method == "fit" and s.yaw is None and math.hypot(*(s.centre[:2] - g[:2])) < 0.02
    cone = L.locate(d, boxes[0], K, T, "traffic_cone", plane)        # same pixels, class without a footprint model
    assert cone.method == "ray" and cone.yaw is None
    off = L.locate(d, boxes[0], K, T, "fire_extinguisher", plane, use_shape=False)
    assert off.method == "ray"


def test_locate_returns_none_for_floor_only_boxes():
    d, boxes, T = shot([CUBE])
    plane = L.FloorTracker(L.FloorParams()).update(d, K, T, 0.0)
    assert L.locate(d, (150, 225, 170, 239), K, T, "box", plane) is None      # floor strip in front of the robot


def test_locate_any_robot_pose_same_map_estimate():
    g = gt_map(TOOLBOX["centre"])
    for (x, y, yaw) in [(0.0, 0.0, 0.0), (0.2, -0.6, 0.45), (2.3, 0.3, math.pi), (1.1, 1.2, -math.pi / 2)]:
        d, boxes, T = shot([TOOLBOX], x, y, yaw)
        if boxes[0] is None:
            continue
        plane = L.FloorTracker(L.FloorParams()).update(d, K, T, 0.0)
        s = L.locate(d, boxes[0], K, T, "toolbox", plane)
        assert s is not None and s.centre[2] == pytest.approx(g[2], abs=0.015), (x, y, yaw)
        assert s.bottom_z == pytest.approx(FLOOR_MAP, abs=0.01)


# ------------------------------------------------------------------ semantic map with 3D tracks
def _sightings(solid, cls, poses, noise=0.003):
    out = []
    for k, (x, y, yaw) in enumerate(poses):
        d, boxes, T = shot([solid], x, y, yaw, noise=noise, seed=k)
        plane = L.FloorTracker(L.FloorParams()).update(d, K, T, 0.0)
        out.append(L.locate(d, boxes[0], K, T, cls, plane))
    return out


def test_semantic_map_json_carries_3d_fields():
    poses = [(0.0, 0.0, 0.0), (0.1, 0.2, -0.1), (0.0, -0.3, 0.2), (0.3, 0.0, 0.0), (-0.2, 0.1, 0.05)]
    m = SemanticMap(min_hits=3)
    for k, s in enumerate(_sightings(CUBE, "box", poses)):
        m.update("box", s.centre, 0.9, float(k), range_m=1.3, obs3d=s)
    (o,) = json.loads(m.to_json())["objects"]
    g = gt_map(CUBE["centre"])
    assert o["pos_source"] == "object3d" and o["hits_3d"] == 5 and o["method_3d"] == "fit"
    assert math.hypot(o["x"] - g[0], o["y"] - g[1]) < 0.02 and o["z"] == pytest.approx(g[2], abs=0.015)
    assert o["top_z"] == pytest.approx(FLOOR_MAP + 0.30, abs=0.015) and o["height"] == pytest.approx(0.30, abs=0.015)
    assert o["bottom_z"] == pytest.approx(FLOOR_MAP, abs=0.005) and o["height_known"]
    assert o["size"][:2] == [0.40, 0.40] and o["size"][2] == pytest.approx(0.30, abs=0.015)
    assert len(o["sigma_xyz"]) == 3 and all(v > 0 for v in o["sigma_xyz"])
    assert (o["yaw"] - CUBE["yaw"]) % (math.pi / 2) < 0.05 or (CUBE["yaw"] - o["yaw"]) % (math.pi / 2) < 0.05


def test_semantic_map_without_3d_is_unchanged():
    m = SemanticMap(min_hits=2)
    for k in range(3):
        m.update("barrel", [1.0 + 0.01 * k, 2.0, 0.3], 0.9, float(k), range_m=2.0)
    (o,) = json.loads(m.to_json())["objects"]
    assert set(o) == {"id", "class", "x", "y", "z", "color", "hits", "score", "sigma_m", "first_seen", "last_seen"}
    t = m.tracks[0]
    assert not t.has3d and np.allclose(t.xyz, t.pos)


def test_pixel_fallback_counts_but_does_not_move_a_3d_track():
    s = _sightings(CUBE, "box", [(0.0, 0.0, 0.0)])[0]
    m = SemanticMap(min_hits=3)
    m.update("box", s.centre, 0.9, 0.0, obs3d=s)
    c = m.tracks[0].xyz.copy()
    m.update("box", s.centre + [0.15, 0.0, 0.2], 0.9, 1.0)              # pixel method sighting, inside the gate
    m.update("box", s.centre + [0.15, 0.0, 0.2], 0.9, 2.0)
    t = m.tracks[0]
    assert t.hits == 3 and t.t3.hits == 1 and np.allclose(t.xyz, c)


def test_truncated_3d_sightings_pull_less():
    a, b = _sightings(CUBE, "box", [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)])
    b.centre = b.centre + [0.1, 0.0, 0.0]
    m = SemanticMap(min_hits=1)
    m.update("box", a.centre, 0.9, 0.0, obs3d=a)
    m.update("box", b.centre, 0.9, 1.0, obs3d=b, weight_scale=0.25)
    x = m.tracks[0].xyz[0]
    assert a.centre[0] < x < a.centre[0] + 0.05                          # equal weights would give + 0.05


def test_merge_of_two_3d_tracks_keeps_the_3d_fusion():
    s1, s2 = _sightings(CUBE, "box", [(0.0, 0.0, 0.0), (0.2, 0.1, 0.0)])
    m = SemanticMap(min_hits=1, gates={"box": 0.10})
    m.update("box", s1.centre, 0.9, 0.0, obs3d=s1)
    m.tracks.append(m.tracks[0].__class__(99, "box", s2.centre.copy(), 100.0, 1, 0.9, 0.9, 1.0, 1.0))
    m.tracks[1].add3d(s2)
    m._merge()
    assert len(m.tracks) == 1 and m.tracks[0].t3.hits == 2 and len(m.tracks[0].floor_zs) == 2


def test_track3d_gate_rejects_a_wrong_3d_sighting():
    ss = _sightings(CUBE, "box", [(0.0, 0.0, 0.0), (0.1, 0.1, 0.0), (0.0, -0.2, 0.1), (0.2, 0.0, 0.0)])
    m = SemanticMap(min_hits=1, gates={"box": 0.5})
    for k, s in enumerate(ss[:3]):
        m.update("box", s.centre, 0.9, float(k), obs3d=s)
    bad = ss[3]
    bad.centre = bad.centre + [0.0, 0.0, 0.25]                          # e.g. top of a stacked object, same xy
    m.update("box", bad.centre, 0.9, 3.0, obs3d=bad)
    t = m.tracks[0]
    assert t.hits == 4 and t.t3.hits == 3 and t.t3.rejected == 1
    assert t.xyz[2] == pytest.approx(gt_map(CUBE["centre"])[2], abs=0.02)


def test_rotated_frame_helper_roundtrip():
    n = rpy_to_R(0.03, -0.02, 0.0) @ np.array([0.0, 0.0, 1.0])
    T = L.floor_transform(n, 0.11)
    p = np.array([0.4, -0.3, 0.0])
    p = p - n * (n @ p + 0.11)                                           # a point on the plane
    assert (T[:3, :3] @ p + T[:3, 3])[2] == pytest.approx(0.0, abs=1e-9)
