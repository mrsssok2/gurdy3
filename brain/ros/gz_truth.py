"""Gazebo ground truth for the fetch retest (2026-11-27): diagnostics only, never used by default.

`GzTruth(world)` follows `gz topic -e -t /world/W/dynamic_pose/info` (the same protobuf-text parser as scripts/grasp_test.py)
and answers `nearest(cls, xy)`: the ground-truth (x, y) of the closest pickable model of that class (models from
docs/results/data/lab4_fetch/lab4_fetch_gt.json). `teleport(x, y, yaw)` = gz set_pose on the robot (it stays standing, as in
grasp_test); with odom_source:=ground_truth the odom frame jumps with it, so the map pose stays right without /initialpose.
Needs the gz CLI (the sim images); runs in the agent container next to Gazebo.
"""
from __future__ import annotations

import json
import math
import subprocess
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GT_FILE = ROOT / "docs/results/data/lab4_fetch/lab4_fetch_gt.json"
ROBOT_MODEL, BASE_Z = "gurdy", 0.115


def models_by_class(gt_file: Path = GT_FILE) -> dict:
    out: dict = {}
    for o in json.loads(Path(gt_file).read_text())["objects"]:
        if o.get("pickable"):
            out.setdefault(o["class"], []).append(o["model"])
    return out


class GzTruth:
    def __init__(self, world: str = "lab4_fetch", gt_file: Path = GT_FILE):
        from gurdy_core import grasp_test as G
        self.world, self.models = world, models_by_class(gt_file)
        self.latest, self.lock = {}, threading.Lock()
        self._parser = G.PoseTextParser()
        self.p = subprocess.Popen(["gz", "topic", "-e", "-t", f"/world/{world}/dynamic_pose/info"], stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL, text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.p.stdout:
            r = self._parser.feed(line)
            if r:
                with self.lock:
                    self.latest[r[0]] = (r[1], r[2], r[3], time.time())

    def get(self, name, max_age=2.0):
        with self.lock:
            v = self.latest.get(name)
        return v if v and time.time() - v[3] <= max_age else None

    def nearest(self, cls: str, xy, max_d: float = 0.6):
        """-> (model, x, y) of the closest model of `cls` to world xy within max_d, else None."""
        best = None
        for m in self.models.get(cls, []):
            v = self.get(m)
            if v is None:
                continue
            d = math.hypot(v[0] - xy[0], v[1] - xy[1])
            if d <= max_d and (best is None or d < best[0]):
                best = (d, m, v[0], v[1])
        return None if best is None else best[1:]

    def has_data(self, cls: str, max_age: float = 2.0) -> bool:
        """True when at least one model of `cls` has a fresh pose (so a None from nearest() means 'not there', not 'no data')."""
        return any(self.get(m, max_age) is not None for m in self.models.get(cls, []))

    def robot(self):
        v = self.get(ROBOT_MODEL)
        return None if v is None else v[:2]

    def teleport(self, x: float, y: float, yaw: float) -> bool:
        req = (f'name: "{ROBOT_MODEL}" position {{x: {x:.4f} y: {y:.4f} z: {BASE_Z:.4f}}} '
               f'orientation {{x: 0 y: 0 z: {math.sin(yaw / 2):.6f} w: {math.cos(yaw / 2):.6f}}}')
        try:
            r = subprocess.run(["gz", "service", "-s", f"/world/{self.world}/set_pose", "--reqtype", "gz.msgs.Pose", "--reptype",
                                "gz.msgs.Boolean", "--timeout", "3000", "--req", req], capture_output=True, text=True, timeout=10)
        except Exception:                                    # noqa: BLE001
            return False
        return r.returncode == 0 and "data: true" in r.stdout

    def close(self):
        self.p.kill()
