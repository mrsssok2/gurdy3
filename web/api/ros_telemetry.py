"""Telemetry from the REAL robot (Gazebo), for the dashboard. Pure Python: it talks to a `RosPort` (brain.ros.port, the same object the agent's
RosBackend uses, so pose / battery / legs / objects are exactly what the agent knows) and to an optional `extras` object for the two things
the port does not carry (planned path, camera frame + detection boxes; `ros_extras.RosExtras` implements it with rclpy).

Unlike MockTelemetry it never takes the agent lock: every read is a plain attribute / topic-cache read, so the stream keeps its 2 Hz while the
LLM is thinking. Nothing is invented: a value with no source yet is None and the page shows a dash.
"""
from __future__ import annotations

import time
from collections import deque
from typing import Any

from .telemetry import moves_of, TrailBuffer

STALE_AFTER_S = 30.0          # same idea as brain.ros.backend.STALE_AFTER_S: a remembered object not seen for this long is drawn faded
PATH_POINTS = 40


def grid_to_rects(width: int, height: int, res: float, ox: float, oy: float, data, thresh: int = 65) -> list[dict]:
    """Occupied cells (value >= thresh) of an occupancy grid -> a short list of rectangles (horizontal runs merged across rows)."""
    rects: list[dict] = []
    open_runs: dict[tuple[int, int], int] = {}       # (x_start, x_end) -> first row of the run
    for j in range(height + 1):
        runs = set()
        if j < height:
            row, i = j * width, 0
            while i < width:
                if data[row + i] >= thresh:
                    k = i
                    while k + 1 < width and data[row + k + 1] >= thresh:
                        k += 1
                    runs.add((i, k))
                    i = k + 1
                else:
                    i += 1
        for run, j0 in list(open_runs.items()):
            if run not in runs:                      # the run ended: emit the block of rows [j0, j)
                rects.append({"x0": ox + run[0] * res, "y0": oy + j0 * res, "x1": ox + (run[1] + 1) * res, "y1": oy + j * res})
                del open_runs[run]
        for run in runs:
            open_runs.setdefault(run, j)
    return [{k: round(v, 3) for k, v in r.items()} for r in rects]


class RosTelemetry:
    def __init__(self, host, port, extras=None, wall=time.monotonic):
        self.host, self.port, self.extras, self.wall = host, port, extras, wall
        self._trail = TrailBuffer()
        self._map: dict[str, Any] | None = None

    @property
    def backend(self):
        return self.host.agent.layer.backend

    def reset(self) -> None:
        self._trail.clear()

    # ------------------------------------------------------------------ live state
    def snapshot(self) -> dict[str, Any]:
        pose = self.port.pose()
        if pose is None:
            return {"connected": False, "stale": False, "source": "ros", "error": "waiting for the map -> base_link transform"}
        b = self.backend
        try:
            st = b.status().data
        except Exception as e:                       # noqa: BLE001  (a backend hiccup must not kill the stream)
            return {"connected": False, "stale": False, "source": "ros", "error": f"status failed: {e}"}
        x, y, yaw = pose
        self._trail.add(x, y)
        now = self.port.now()
        objs = []
        for t in self.port.semantic_objects() or []:
            age = now - t.get("last_seen", now)
            objs.append({"id": t.get("id"), "class": t.get("class"), "color": t.get("color") or "", "x": round(t["x"], 2), "y": round(t["y"], 2),
                         "place": b._place_at(t["x"], t["y"]) if hasattr(b, "_place_at") else None, "age_s": round(age, 1), "stale": age > STALE_AFTER_S})
        import math
        return {
            "connected": True, "stale": False, "source": "ros", "sim_time_s": round(now, 1),
            "pose": {"x": round(x, 3), "y": round(y, 3), "heading_deg": round(math.degrees(yaw), 1),
                     "place": b._place_at(x, y) if hasattr(b, "_place_at") else None},
            "mode": st["mode"], "moving": st["mode"] == "moving", "estop": st["estop"], "task": st["task"],
            "speed_mps": st["speed_mps"], "battery_pct": st["battery_pct"], "battery_simulated": st["battery_simulated"],
            "legs": st["legs"], "alerts": list(st["alerts_active"]), "policy": st.get("policy"),
            "objects": objs, "zones": [dict(z) for z in getattr(b, "zones", [])],
            "trail": self._trail.points(), "path": self._planned_path(st),
            "camera_available": bool(self.extras and self.extras.has_camera()),
            "moves": moves_of(b),
        }

    def _planned_path(self, st) -> list[list[float]]:
        if not st.get("task") or self.extras is None:
            return []
        pts = self.extras.plan()
        if len(pts) > PATH_POINTS:
            step = len(pts) / PATH_POINTS
            pts = [pts[int(i * step)] for i in range(PATH_POINTS)] + [pts[-1]]
        return [[round(p[0], 2), round(p[1], 2)] for p in pts]

    # ------------------------------------------------------------------ static geometry (sent once, re-read until the /map exists)
    def map(self) -> dict[str, Any]:
        if self._map is not None:
            return self._map
        b = self.backend
        rooms = {n: {"x0": v["rect"][0], "y0": v["rect"][1], "x1": v["rect"][2], "y1": v["rect"][3]}
                 for n, v in getattr(b, "places", {}).items() if "rect" in v}
        places = {n: {"x": v["x"], "y": v["y"]} for n, v in getattr(b, "places", {}).items()}
        occ = self.port.occupancy() if hasattr(self.port, "occupancy") else None
        walls = grid_to_rects(*occ) if occ else []
        if rooms:
            pad = 0.3
            bounds = {"x0": min(r["x0"] for r in rooms.values()) - pad, "y0": min(r["y0"] for r in rooms.values()) - pad,
                      "x1": max(r["x1"] for r in rooms.values()) + pad, "y1": max(r["y1"] for r in rooms.values()) + pad}
        elif occ:
            w, h, res, ox, oy, _ = occ
            bounds = {"x0": ox, "y0": oy, "x1": ox + w * res, "y1": oy + h * res}
        else:
            bounds = {"x0": -3.0, "y0": -3.0, "x1": 3.0, "y1": 3.0}
        m = {"map_id": self.port.map_id(), "bounds": bounds, "rooms": rooms, "walls": walls, "obstacles": [], "places": places}
        if occ:                       # cache only once the real map exists; before that the page gets the rooms and asks again on reconnect
            self._map = m
        return m

    # ------------------------------------------------------------------ camera
    def camera(self) -> dict[str, Any] | None:
        return self.extras.camera() if self.extras is not None else None
