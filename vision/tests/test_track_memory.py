"""detector_node unconfirmed-track memory (ttl) = 30 s (2026-11-28), checked on the committed fetch sighting logs of that day:
the red cube (11 good sightings in two passes 29 s apart) is confirmed with 30 s and NOT with the old 10 s, and the
longer memory adds no false track (vision/replay_fetch_sightings.py)."""
import importlib.util
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
RUNS = [ROOT / "docs/results/data/smoke_1128b" / r for r in ("can_1", "can_2", "can_3", "cube")] + \
       [ROOT / "docs/results/data/smoke_1128/can_1"]


def node_default(name):
    src = (ROOT / "ros2_ws/src/gurdy_perception/gurdy_perception/detector_node.py").read_text()
    return float(re.search(rf'P\("{name}", ([0-9.]+)\)', src).group(1))


def replay(ttl):
    spec = importlib.util.spec_from_file_location("rfs", ROOT / "vision/replay_fetch_sightings.py")
    R = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(R)
    from gurdy_core.vision.semantic_map import SemanticMap
    out = {}
    for k, (run, t, log) in enumerate(R.runs(RUNS)):
        m = SemanticMap(**dict(R.NODE, ttl=ttl, min_hits=int(node_default("min_hits"))))
        for s in log:
            m.update(s["cls"], s["p"], s["score"], s["t"], range_m=s["range"], color=s.get("color", ""),
                     weight_scale=R.TRUNC_W if s.get("truncated") else 1.0)
        out[f"{k}:{run}/{t['id']}"] = R.grade(m, R.ground_truth(t))
    return out


def test_node_default_is_30_s_and_launch_passes_it():
    assert node_default("ttl") == 30.0
    launch = (ROOT / "ros2_ws/src/gurdy_bringup/launch/sim.launch.py").read_text()
    assert 'DeclareLaunchArgument("detector_ttl", default_value="30.0"' in launch and '"ttl": PythonExpression' in launch


@pytest.mark.skipif(not all((r / "fetch/tries.jsonl").exists() for r in RUNS), reason="smoke logs not in this checkout")
def test_replay_cube_confirmed_with_30_s_without_new_false_tracks():
    old, new = replay(10.0), replay(node_default("ttl"))
    assert len(new) == 5
    cube = "3:cube/small_box_1"
    assert not old[cube]["target"] and new[cube]["target"]
    assert new[cube]["target_label"] == "red small_box"
    assert all(g["target"] for g in new.values())                          # every try's target confirmed
    assert sum(g["false"] for g in new.values()) == sum(g["false"] for g in old.values())
