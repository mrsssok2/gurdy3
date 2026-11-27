"""Phase 8c Gazebo accuracy check: stop plan geometry + scorer on a synthetic run (no ROS, no Gazebo)."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "vision"))
sys.path.insert(0, str(ROOT / "gurdy_core"))
import accuracy3d as A  # noqa: E402
from gurdy_core.vision import locate3d as L  # noqa: E402
from gurdy_core.vision.geometry import backproject, box_depth, surface_to_centre  # noqa: E402
from gurdy_core.vision.classes import radius_prior  # noqa: E402
from gurdy_core.vision.synth_depth import render  # noqa: E402
from gurdy_core.vision.transforms import T_base_optical, T_map_base, apply_T, intrinsics_from_fov, make_T  # noqa: E402

CAM = {"pos": [0.15, 0.0, 0.02], "pitch_down": 0.17}
K = intrinsics_from_fov(320, 240, 1.047)


def test_plan_covers_every_object_range_and_bearing_with_clearance():
    gt = A.load_gt()
    plan = A.plan_stops(gt)
    assert len(gt) == 11 and not plan["skipped"]
    assert len(plan["stops"]) == 11 * len(A.RANGES) * len(A.BEARINGS_DEG)
    obs = A.obstacles(gt)
    for s in plan["stops"]:
        assert s["foot_clearance"] >= A.MARGIN
        assert min(A._dist_point(o, s["x"], s["y"]) for o in obs) - A.FOOT_R >= A.MARGIN - 1e-9
        g = next(h for h in gt if h["name"] == s["name"])
        cx, cy = s["x"] + A.CAM_X * math.cos(s["yaw"]), s["y"] + A.CAM_X * math.sin(s["yaw"])
        assert math.hypot(g["x"] - cx, g["y"] - cy) == pytest.approx(s["range"], abs=1e-3)
        rel = math.atan2(g["y"] - cy, g["x"] - cx) - s["yaw"]
        assert math.degrees(math.atan2(math.sin(rel), math.cos(rel))) == pytest.approx(s["bearing_deg"], abs=0.01)


def test_obstacles_include_walls_furniture_and_objects_once():
    names = [o[0] for o in A.obstacles(A.load_gt())]
    assert "table" in names and "green_cylinder" in names and "wall_south" in names and "divider_low" in names
    assert names.count("red_box") == 1


_solid = A._solid


def _record(g, st, t, seed):
    """One detector_node log record (position_method object3d) for the stop's object, rendered synthetically."""
    Tw = T_map_base(st["x"], st["y"], st["yaw"], z=0.11) @ T_base_optical(CAM)
    d, _, boxes = render(K, 320, 240, Tw, [_solid(g)], noise=0.003, rng=np.random.default_rng(seed))
    T = make_T(t=(0, 0, -0.11)) @ Tw                               # odom frame: floor at -0.11
    s3 = L.locate(d, boxes[0], K, T, g["class"], L.FloorTracker().update(d, K, T, t))
    z, _ = box_depth(d, boxes[0])
    u, v = 0.5 * (boxes[0][0] + boxes[0][2]), 0.5 * (boxes[0][1] + boxes[0][3])
    p_pix = apply_T(T, surface_to_centre(backproject(u, v, z, K), radius_prior(g["class"]))[None, :])[0]
    return {"t": t, "cls": g["class"], "score": 0.9, "p": [float(c) for c in s3.centre], "range": 1.0, "color": "",
            "truncated": False, "o3d": s3.to_dict(), "p_pixel": [float(c) for c in p_pix]}


def test_score_synthetic_run(tmp_path):
    gt = A.load_gt()
    plan = A.plan_stops(gt)
    pick = [s for s in plan["stops"] if s["name"] in ("red_box", "extinguisher") and s["range"] in (1.0, 1.5) and s["bearing_deg"] == 0]
    stops, recs, t = [], [], 10.0
    for st in pick:
        g = next(h for h in gt if h["name"] == st["name"])
        stops.append({**st, "valid": True, "t0": t, "t1": t + 2.0})
        recs += [_record(g, st, t + 0.2 * k, k) for k in range(5)]
        recs.append({**recs[-1], "t": t + 3.0})                    # between stops: ignored
        t += 10.0
    stops.append({**plan["stops"][0], "valid": False, "reason": "fallen / tilted", "t0": t, "t1": t + 2})
    (tmp_path / "stops.json").write_text(json.dumps({"stops": stops}))
    (tmp_path / "sightings.jsonl").write_text("\n".join(json.dumps(r) for r in recs) + "\n")
    res = A.score(tmp_path, gt)
    assert res["n_target"] == 20 and res["n_sightings"] == 24 and res["invalid_stops"] == [plan["stops"][0]["id"]]
    for row in res["table"]:
        assert row["fallback"] == 0 and row["method"] == ["fit"]
        assert row["xy_o3d"]["median"] < 0.04, row            # synthetic, size-adaptive box fit / circle fit
        assert row["z"] is None or row["z"]["median"] < 0.02                # top known
    box10 = next(r for r in res["table"] if r["class"] == "box" and r["range"] == "1.0")
    assert box10["top_known"] == 5 and box10["false_top"] == 0 and box10["top"]["median"] < 0.015
    assert res["floor_z"]["median"] == pytest.approx(-0.11, abs=0.005)
    fused = [s["fused"] for s in res["stops"] if s["fused"]]
    assert len(fused) == 4 and all(f["xy"] < 0.04 and f["hits"] == 5 for f in fused)
    md = A.report_md(res)
    assert "| box | 1.0 | 5 | 0 | fit |" in md and "top cut" in md and "false top claims 0" in md


def test_score_compares_the_semantic_map_above_the_floor(tmp_path):
    gt = A.load_gt()
    (tmp_path / "stops.json").write_text(json.dumps({"stops": []}))
    (tmp_path / "sightings.jsonl").write_text("")
    g = next(h for h in gt if h["name"] == "red_box")
    sem = {"objects": [{"class": "box", "x": g["x"] + 0.01, "y": g["y"], "z": g["z"] - 0.11, "top_z": g["z"] + 0.15 - 0.11,
                        "height": 0.30}]}
    (tmp_path / "semantic.json").write_text(json.dumps(sem))
    res = A.score(tmp_path, gt)
    sm = res["semantic_map"]
    assert sm["false"] == 0 and sm["xy_err"]["median"] == pytest.approx(0.01, abs=1e-6)
    assert sm["z_err"]["median"] == pytest.approx(0.0, abs=1e-6) and sm["top_err"]["median"] == pytest.approx(0.0, abs=1e-6)


def test_synth_dry_run_fit_beats_the_pixel_method_where_the_shape_is_observable():
    rows = {r["name"]: r for r in A.synth(seeds=1)}
    assert set(rows) == {"barrel_a", "barrel_b", "extinguisher", "toolbox", "cardboard_box", "red_box", "blue_box", "yellow_box"}
    for n in ("barrel_a", "extinguisher", "toolbox", "cardboard_box", "yellow_box"):
        assert rows[n]["fit"]["median"] < rows[n]["pixel"]["median"], n
    assert rows["barrel_a"]["top"] is None                                 # never visible from the body camera
