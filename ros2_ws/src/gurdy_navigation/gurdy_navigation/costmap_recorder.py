"""Costmap snapshots at planner failures (Phase 8a cause 2: planner failures from free poses that a global-costmap clear
removes; the marks were never recorded). Pure Python + numpy, no ROS imports: used by costmap_recorder_node, unit-tested.

A failure is a planner_server log line such as
    GridBased plugin failed to plan from (2.32, -0.69) to (0.00, 4.00): "Failed to create plan with tolerance of: 0.300000"
The node keeps the last two global costmaps it received (published ~1 Hz, so they are from BEFORE the BT's / the agent's
clear, which comes ~20 ms after the log line) and writes them with the robot pose.

Files per failure (prefix fail_<n>_<wall time>): .npz (costmaps as int8 arrays + origin / resolution / stamps / poses / text),
.json (metadata only) and .pgm (latest costmap as an image: free white, unknown grey, cost darker, lethal black; robot start
and goal marked) - viewable without any tool.
"""
from __future__ import annotations

import json
import math
import re
import time
from pathlib import Path

import numpy as np

FAIL_RE = re.compile(r"failed to plan from \((-?[\d.]+), (-?[\d.]+)\) to \((-?[\d.]+), (-?[\d.]+)\)", re.I)
FAIL_TEXTS = ("Failed to create plan", "failed to plan from", "No valid path", "Planner timed out")


def parse_failure(logger_name: str, text: str) -> dict | None:
    """-> {'start': (x, y) | None, 'goal': (x, y) | None, 'text': text} for a planner failure line, else None."""
    if "planner_server" not in (logger_name or "") and "planner" not in (logger_name or ""):
        return None
    if not any(t.lower() in text.lower() for t in FAIL_TEXTS):
        return None
    m = FAIL_RE.search(text)
    if m:
        x0, y0, x1, y1 = (float(v) for v in m.groups())
        return {"start": (x0, y0), "goal": (x1, y1), "text": text}
    return {"start": None, "goal": None, "text": text}


def grid_to_pgm(data: np.ndarray, marks=()) -> bytes:
    """OccupancyGrid data (H x W int8: -1 unknown, 0..100 cost) -> binary PGM, row 0 = top (max y). marks = [(row, col)]."""
    d = data.astype(np.int16)
    img = np.where(d < 0, 160, 255 - np.clip(d, 0, 100) * 255 // 100).astype(np.uint8)
    img[d >= 99] = 0
    for r, c in marks:
        if 0 <= r < img.shape[0] and 0 <= c < img.shape[1]:
            img[max(r - 1, 0):r + 2, max(c - 1, 0):c + 2] = 90
    img = img[::-1]
    h, w = img.shape
    return f"P5\n{w} {h}\n255\n".encode() + img.tobytes()


def world_to_cell(x: float, y: float, origin, res: float) -> tuple:
    return int(math.floor((y - origin[1]) / res)), int(math.floor((x - origin[0]) / res))


def lethal_near(data: np.ndarray, origin, res: float, xy, radius: float) -> int:
    """Number of cells with cost >= 99 (inscribed / lethal) within `radius` m of xy: the 'mark' a free pose should not have."""
    r0, c0 = world_to_cell(xy[0], xy[1], origin, res)
    k = int(math.ceil(radius / res))
    sub = data[max(r0 - k, 0):r0 + k + 1, max(c0 - k, 0):c0 + k + 1]
    rr, cc = np.mgrid[max(r0 - k, 0):max(r0 - k, 0) + sub.shape[0], max(c0 - k, 0):max(c0 - k, 0) + sub.shape[1]]
    inside = (rr - r0) ** 2 + (cc - c0) ** 2 <= k * k
    return int(np.count_nonzero((sub >= 99) & inside))


def save_snapshot(out_dir, n: int, failure: dict, grids: list, robot_pose, wall_t: float | None = None) -> dict:
    """grids: [{'data': HxW int8, 'origin': (x, y), 'res': m, 'stamp': s}, ...] newest LAST. Writes .npz/.json/.pgm."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    wall_t = time.time() if wall_t is None else wall_t
    stem = out / f"fail_{n:03d}_{time.strftime('%H%M%S', time.gmtime(wall_t))}"
    meta = {"n": n, "wall_time": wall_t, "text": failure["text"], "start": failure.get("start"), "goal": failure.get("goal"),
            "robot_pose": list(robot_pose) if robot_pose is not None else None, "grids": []}
    arrays = {}
    for i, g in enumerate(grids):
        arrays[f"grid{i}"] = np.asarray(g["data"], dtype=np.int8)
        gm = {"origin": list(g["origin"]), "res": g["res"], "stamp": g["stamp"], "shape": list(np.shape(g["data"]))}
        if failure.get("start"):
            gm["lethal_cells_within_0.3m_of_start"] = lethal_near(arrays[f"grid{i}"], g["origin"], g["res"], failure["start"], 0.3)
        meta["grids"].append(gm)
    np.savez_compressed(f"{stem}.npz", meta=json.dumps(meta), **arrays)
    Path(f"{stem}.json").write_text(json.dumps(meta, indent=1))
    if grids:
        g = grids[-1]
        marks = [world_to_cell(*p, g["origin"], g["res"]) for p in (failure.get("start"), failure.get("goal")) if p]
        Path(f"{stem}.pgm").write_bytes(grid_to_pgm(np.asarray(g["data"], dtype=np.int8), marks))
    meta["files"] = [f"{stem}.npz", f"{stem}.json", f"{stem}.pgm"]
    return meta
