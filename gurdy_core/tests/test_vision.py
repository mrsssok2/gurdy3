"""Phase 4 perception core: classes, YOLO decoding/NMS, depth geometry, semantic map, metrics, colours."""
import numpy as np
import pytest

from gurdy_core.vision import classes as C
from gurdy_core.vision.colors import color_name, resolve_color
from gurdy_core.vision.detector import decode_yolo, iou_xyxy, letterbox, nms, preprocess
from gurdy_core.vision.geometry import K_from_camera_info, backproject, box_depth, quat_to_R, surface_to_centre, transform_point
from gurdy_core.vision.metrics import average_precision, evaluate, match
from gurdy_core.vision.geometry import truncation_flags, visible_centre_u
from gurdy_core.vision.semantic_map import SemanticMap, obs_sigma


def test_class_tables_consistent():
    assert len(C.CLASSES) == 7 == len(set(C.CLASSES))
    for c in C.CLASSES:
        assert c in C.ALIASES and c in C.SIZE_PRIOR and c in C.COLORS and C.radius_prior(c) > 0


def test_resolve_class():
    assert C.resolve_class("find the red box") == "box"
    assert C.resolve_class("fire extinguisher") == "fire_extinguisher"
    assert C.resolve_class("go to the chair") == "chair"
    assert C.resolve_class("anything") is None


def test_colors():
    assert color_name(np.full((4, 4, 3), (220, 20, 20))) == "red"
    assert color_name(np.full((4, 4, 3), (20, 40, 210))) == "blue"
    assert color_name(np.full((4, 4, 3), (230, 210, 30))) == "yellow"
    assert color_name(np.full((4, 4, 3), (10, 10, 10))) == "black"
    assert resolve_color("red box") == "red" and resolve_color("blue barrel") == "blue" and resolve_color("box") is None


def test_letterbox_roundtrip_geometry():
    img = np.zeros((240, 320, 3), np.uint8)
    out, s, pad = letterbox(img, (256, 320))
    assert out.shape == (256, 320, 3) and s == 1.0 and pad == (0, 8)
    out, s, pad = letterbox(img, (320, 320))
    assert out.shape == (320, 320, 3) and s == 1.0 and pad == (0, 40)
    x, _, _ = preprocess(img, (256, 320))
    assert x.shape == (1, 3, 256, 320) and x.dtype == np.float32


def _fake_output(rows, nc=7, n=50):
    """(1, 4+nc, n) YOLO head output with the given (cx, cy, w, h, cls, score) rows, rest zero."""
    out = np.zeros((4 + nc, n), np.float32)
    for i, (cx, cy, w, h, c, s) in enumerate(rows):
        out[:4, i] = (cx, cy, w, h)
        out[4 + c, i] = s
    return out[None]


def test_decode_maps_back_to_image_and_applies_nms():
    # image 240x320 letterboxed to 256x320: pad_y = 8, scale 1. Box centred at (160, 128) size 40x60 in input px.
    rows = [(160, 128, 40, 60, 3, 0.9), (162, 129, 40, 60, 3, 0.6), (50, 50, 20, 20, 1, 0.2), (250, 200, 30, 30, 5, 0.8)]
    dets = decode_yolo(_fake_output(rows), 1.0, (0, 8), (240, 320), conf_thr=0.35, iou_thr=0.5)
    assert [d.name for d in dets] == ["traffic_cone", "mannequin"]          # duplicate suppressed, low score dropped
    assert dets[0].box == pytest.approx((140, 90, 180, 150))                # y shifted by the padding
    assert dets[0].score == pytest.approx(0.9)


def test_decode_clips_and_accepts_transposed_layout():
    d = decode_yolo(_fake_output([(5, 100, 40, 40, 0, 0.9)]).transpose(0, 2, 1), 1.0, (0, 0), (240, 320))
    assert d[0].box[0] == 0.0
    with pytest.raises(ValueError):
        decode_yolo(np.zeros((1, 9, 9), np.float32), 1.0, (0, 0), (240, 320))


def test_nms_is_class_aware_in_decode():
    rows = [(100, 100, 40, 40, 0, 0.9), (100, 100, 40, 40, 1, 0.8)]       # same box, different class: both kept
    assert len(decode_yolo(_fake_output(rows), 1.0, (0, 0), (240, 320))) == 2
    b = np.array([[0, 0, 10, 10], [1, 1, 10, 10], [20, 20, 30, 30]], float)
    assert nms(b, np.array([0.9, 0.8, 0.7]), 0.5) == [0, 2]
    assert iou_xyxy(b[:1], b[:1])[0, 0] == pytest.approx(1.0)


def test_box_depth_ignores_background_and_invalid():
    depth = np.full((240, 320), 4.0, np.float32)
    depth[80:160, 120:200] = 1.5                    # object
    depth[100:105, 150:155] = np.nan
    z, frac = box_depth(depth, (110, 70, 210, 170))
    assert z == pytest.approx(1.5, abs=1e-6) and frac > 0.9
    assert box_depth(np.zeros((240, 320), np.float32), (10, 10, 50, 50))[0] is None
    assert box_depth(depth, (-50, -50, -10, -10))[0] is None


def test_backproject_center_and_offsets():
    K = K_from_camera_info([277.0, 0, 160.0, 0, 277.0, 120.0, 0, 0, 1])
    assert backproject(160, 120, 2.0, K) == pytest.approx([0, 0, 2.0])
    p = backproject(160 + 277, 120, 2.0, K)
    assert p == pytest.approx([2.0, 0, 2.0])
    c = surface_to_centre(np.array([0, 0, 2.0]), 0.2)
    assert c == pytest.approx([0, 0, 2.2])


def test_transform_point_yaw_90():
    q = [0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)]
    assert transform_point([1, 0, 0], [1, 2, 3], q) == pytest.approx([1, 3, 3])
    assert quat_to_R([0, 0, 0, 1]) == pytest.approx(np.eye(3))


def test_semantic_map_dedupes_repeated_sightings():
    m = SemanticMap(min_hits=3)
    rng = np.random.default_rng(0)
    for k in range(20):
        m.update("barrel", [2.0, 1.0, 0.4] + rng.normal(0, 0.03, 3), 0.8, float(k), range_m=1.5, color="blue")
    assert len(m.tracks) == 1 and len(m.confirmed()) == 1
    t = m.confirmed()[0]
    assert t.hits == 20 and np.linalg.norm(t.pos[:2] - [2.0, 1.0]) < 0.03 and t.color == "blue"


def test_semantic_map_separates_instances_and_classes():
    m = SemanticMap(min_hits=2)
    for k in range(3):
        m.update("barrel", [0, 0, 0], 0.9, k)
        m.update("barrel", [3, 0, 0], 0.9, k)       # second barrel of the same class
        m.update("chair", [0.05, 0, 0], 0.9, k)     # other class at (almost) the same place
    assert sorted(t.cls for t in m.confirmed()) == ["barrel", "barrel", "chair"]
    assert len(m.find("barrel", near=[2.9, 0])) == 2 and m.find("barrel", near=[2.9, 0])[0].pos[0] == pytest.approx(3, abs=0.01)


def test_semantic_map_one_off_false_positive_never_confirmed_and_expires():
    m = SemanticMap(min_hits=3, ttl=5.0)
    m.update("toolbox", [1, 1, 0], 0.5, 0.0)
    assert m.confirmed() == []
    m.update("chair", [4, 4, 0], 0.9, 10.0)         # 10 s later: the unconfirmed toolbox is dropped
    assert all(t.cls != "toolbox" for t in m.tracks)


def test_semantic_map_near_sighting_outweighs_far_and_merge():
    m = SemanticMap(min_hits=1)
    m.update("box", [1.0, 0, 0], 0.9, 0.0, range_m=0.8)
    m.update("box", [1.2, 0, 0], 0.9, 1.0, range_m=4.0)
    assert m.tracks[0].pos[0] == pytest.approx(1.0, abs=0.03) and obs_sigma(4.0) > 5 * obs_sigma(0.8)
    m2 = SemanticMap(min_hits=1, gates={"box": 0.3})
    m2.tracks = []
    m2.update("box", [0, 0, 0], 0.9, 0)
    m2.update("box", [0.35, 0, 0], 0.9, 1)          # outside the 0.3 gate: second track
    assert len(m2.tracks) == 2
    m2.update("box", [0.17, 0, 0], 0.9, 2, range_m=0.5)   # lands inside the gate of both -> tracks pulled together/merged
    assert len(m2.tracks) <= 2


def test_semantic_map_json_roundtrip():
    import json
    m = SemanticMap(min_hits=1)
    m.update("chair", [1, 2, 0.4], 0.7, 3.0, color="brown")
    o = json.loads(m.to_json())["objects"][0]
    assert o["class"] == "chair" and o["x"] == 1 and o["color"] == "brown"


def test_ap_and_matching():
    assert average_precision(np.array([0.9, 0.8]), np.array([1, 1]), 2) == pytest.approx(1.0)
    assert average_precision(np.array([0.9, 0.8]), np.array([0, 0]), 2) == 0.0
    assert np.isnan(average_precision(np.array([]), np.array([]), 0))
    m, miss = match([(0.9, (0, 0, 10, 10)), (0.8, (0, 0, 10, 10))], [(0, 0, 10, 10)])
    assert [t for _, t in m] == [True, False] and miss == 0            # a GT can be matched once
    m, miss = match([(0.9, (50, 50, 60, 60))], [(0, 0, 10, 10)])
    assert m == [(0.9, False)] and miss == 1


def test_evaluate_precision_recall():
    imgs = [{"pred": [(0, 0.9, (0, 0, 10, 10)), (1, 0.9, (20, 20, 30, 30))], "gt": [(0, (0, 0, 10, 10))]},
            {"pred": [], "gt": [(0, (5, 5, 15, 15))]}]
    r = evaluate(imgs, 2)
    assert r[0]["recall"] == 0.5 and r[0]["precision"] == 1.0 and r[1]["n_gt"] == 0 and np.isnan(r[1]["ap"])


def test_visible_centre_u_truncation():
    W, H, fx, r, z = 320, 240, 277.0, 0.2, 2.0
    half = fx * r / z                                           # 27.7 px apparent radius
    assert visible_centre_u((100, 50, 160, 150), W, H, z, fx, r) == 130           # not truncated: box centre
    assert abs(visible_centre_u((0, 50, 40, 150), W, H, z, fx, r) - (40 - half)) < 1e-9     # cut on the left
    assert abs(visible_centre_u((280, 50, 320, 150), W, H, z, fx, r) - (280 + half)) < 1e-9  # cut on the right
    assert visible_centre_u((100, 0, 160, 150), W, H, z, fx, r) == 130            # top-cut: horizontal centre unchanged
    assert visible_centre_u((0, 50, 320, 150), W, H, z, fx, r) is None
    assert truncation_flags((0, 0, 50, 240), W, H) == {"left": True, "right": False, "top": True, "bottom": True}


def test_truncated_sighting_weighs_less():
    m = SemanticMap(min_hits=1)
    m.update("box", [1.0, 0.0, 0.2], 0.9, 0.0, range_m=1.0)
    m.update("box", [1.2, 0.0, 0.2], 0.9, 1.0, range_m=1.0, weight_scale=0.25)
    assert 1.0 < m.tracks[0].pos[0] < 1.05                      # pulled 1/5 of the way, not halfway


def test_confirmation_needs_span_score_and_consistency():
    kw = dict(min_hits=3, ttl=30.0, min_span=1.0, min_mean_score=0.5, max_scatter=0.5)
    # three sightings in 0.4 s (consecutive frames of one artefact) -> not confirmed; a later one completes the span
    m = SemanticMap(**kw)
    for k in range(3):
        m.update("chair", [2.0, 1.0, 0.3], 0.9, 0.2 * k)
    assert m.confirmed() == []
    m.update("chair", [2.0, 1.0, 0.3], 0.9, 1.3)
    assert len(m.confirmed()) == 1
    # low mean score -> never confirmed however often it is seen
    m = SemanticMap(**kw)
    for k in range(10):
        m.update("box", [0.0, 0.0, 0.1], 0.4, 0.5 * k)
    assert m.confirmed() == []
    # sightings that wander inside the gate (inconsistent position) are not confirmed
    m = SemanticMap(**kw, gates={"toolbox": 0.5})
    for k, x in enumerate([0.0, 0.45, -0.1, 0.4, 0.0, 0.45]):
        m.update("toolbox", [x, 0.0, 0.1], 0.9, 0.5 * k)
    assert m.confirmed() == []
    # once confirmed it stays confirmed
    m = SemanticMap(**kw)
    for k in range(4):
        m.update("barrel", [1.0, 1.0, 0.4], 0.9, 0.5 * k)
    assert len(m.confirmed()) == 1
    m.update("barrel", [1.2, 1.0, 0.4], 0.3, 9.0)
    assert len(m.confirmed()) == 1


def test_dominance_suppresses_sparse_class_confusion_on_a_well_seen_object():
    import numpy as np
    from gurdy_core.vision.semantic_map import SemanticMap
    def feed(m):
        for i in range(60):
            m.update("box", [1.0, 1.6, 0.15], 0.9, i * 0.5)
        for i in range(8):                      # the same spot, a different class, 8 sightings
            m.update("fire_extinguisher", [1.05, 1.62, 0.3], 0.7, i * 0.5)
        m.update("traffic_cone", [3.0, 0.0, 0.2], 0.9, 1.0)
        for i in range(1, 10):
            m.update("traffic_cone", [3.0, 0.0, 0.2], 0.9, 1.0 + i)
    off = SemanticMap(min_hits=5, min_span=2.0, min_mean_score=0.5)
    feed(off)
    assert {t.cls for t in off.confirmed()} == {"box", "fire_extinguisher", "traffic_cone"}
    on = SemanticMap(min_hits=5, min_span=2.0, min_mean_score=0.5, dominance_ratio=4.0)
    feed(on)
    assert {t.cls for t in on.confirmed()} == {"box", "traffic_cone"}      # far-away real object unaffected


def test_color_from_saturated_pixels_only():
    """2026-11-28: a white band / grey floor around a coloured object must not name it (old median rule: 'white')."""
    from gurdy_core.vision.colors import color_name_v1
    crop = np.full((10, 10, 3), (235, 235, 235), np.uint8)       # 70 % white band / highlight
    crop[:2] = (120, 120, 125)                                    # 20 % grey floor
    crop[-1] = (255, 115, 5)                                      # 10 %... below the 15 % threshold: genuinely white
    assert color_name(crop) == "white" and color_name_v1(crop) == "white"
    crop[-3:] = (255, 115, 5)                                     # 30 % orange: the object's colour wins over the band
    assert color_name(crop) == "orange" and color_name_v1(crop) != "orange"
    assert color_name(np.full((4, 4, 3), (194, 196, 204))) in ("gray", "white")   # silver can: unsaturated fallback
    assert color_name(np.full((4, 4, 3), (128, 128, 128))) == "gray"
    red = np.full((6, 6, 3), (230, 20, 20), np.uint8)
    red[:3, :3] = (240, 20, 60)                                   # hue wraps around 0/360: still red, not pink
    assert color_name(red) == "red"


def test_color_rule_not_worse_on_the_labelled_test_images():
    """Per-class accuracy on the rendered synth_v3 test splits (vision/color_eval.py); skipped without the datasets."""
    import importlib.util
    from collections import defaultdict
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    D = root / "datasets/synth_v3_wrist"
    if not (D / "meta/test").exists():
        pytest.skip("datasets/synth_v3_wrist not rendered here")
    spec = importlib.util.spec_from_file_location("color_eval", root / "vision/color_eval.py")
    CE = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(CE)
    from gurdy_core.vision.colors import color_name_v1
    n, old, new = defaultdict(int), defaultdict(int), defaultdict(int)
    for cls, col, img, box in CE.instances(D, "test"):
        exp, c = CE.EXPECT.get(col, {col}), CE.crop_of(img, box)
        n[cls] += 1
        old[cls] += color_name_v1(c) in exp
        new[cls] += color_name(c) in exp
    for cls in n:
        assert new[cls] >= old[cls], cls
    assert sum(new.values()) / sum(n.values()) >= 0.75
