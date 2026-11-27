"""Phase 8a cause 2: costmap snapshot at planner failures (pure part, no ROS)."""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gurdy_navigation.costmap_recorder import grid_to_pgm, lethal_near, parse_failure, save_snapshot, world_to_cell  # noqa: E402

# exact lines from brain/runs/doorway_v1/run2/logs/dw_tight_corner.attempt1.stack/launch.log (planner_server)
REAL = 'GridBased plugin failed to plan from (2.32, -0.69) to (0.00, 4.00): "Failed to create plan with tolerance of: 0.300000"'


def test_parses_the_real_failure_line():
    f = parse_failure("planner_server", REAL)
    assert f["start"] == (2.32, -0.69) and f["goal"] == (0.0, 4.0)


def test_other_lines_and_other_nodes_are_ignored():
    assert parse_failure("planner_server", "Planner loop missed its desired rate of 20.0000 Hz") is None
    assert parse_failure("controller_server", REAL) is None
    assert parse_failure("bt_navigator", "Goal succeeded") is None
    f = parse_failure("planner_server", "Failed to create plan with tolerance of: 0.300000")
    assert f is not None and f["start"] is None


def _grid():
    d = np.zeros((40, 60), dtype=np.int8)       # 0.05 m cells, origin (-1, -1): covers x -1..2, y -1..1
    d[20, 30] = 100                              # lethal cell at x 0.525, y 0.025
    d[0:5, 0:5] = -1
    return {"data": d, "origin": (-1.0, -1.0), "res": 0.05, "stamp": 12.5}


def test_lethal_near_counts_marks_around_the_start():
    g = _grid()
    assert world_to_cell(0.525, 0.025, g["origin"], g["res"]) == (20, 30)
    assert lethal_near(g["data"], g["origin"], g["res"], (0.5, 0.0), 0.3) == 1
    assert lethal_near(g["data"], g["origin"], g["res"], (1.5, 0.0), 0.3) == 0


def test_snapshot_files(tmp_path):
    g = _grid()
    meta = save_snapshot(tmp_path, 1, parse_failure("planner_server", REAL.replace("2.32, -0.69", "0.50, 0.00")),
                         [g, g], (0.5, 0.0, 0.1), wall_t=0.0)
    npz, js, pgm = (Path(f) for f in meta["files"])
    z = np.load(npz)
    assert z["grid1"].shape == (40, 60) and z["grid1"][20, 30] == 100
    m = json.loads(js.read_text())
    assert m["robot_pose"] == [0.5, 0.0, 0.1] and m["grids"][1]["lethal_cells_within_0.3m_of_start"] == 1
    assert pgm.read_bytes().startswith(b"P5\n60 40\n255\n") and len(pgm.read_bytes()) == len(b"P5\n60 40\n255\n") + 2400


def test_pgm_colours():
    img = grid_to_pgm(np.array([[-1, 0, 50, 100]], dtype=np.int8))
    assert list(img[-4:]) == [160, 255, 128, 0]
