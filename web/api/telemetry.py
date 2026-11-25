"""Telemetry sources for the dashboard. A source answers `snapshot()` (small dict, sent ~2 Hz) and `map()` (static geometry, sent once).
`MockTelemetry` reads a MockBackend; the ROS version (rclpy) comes in Phase 7 step 5 and implements the same two methods."""
from __future__ import annotations

import math
import time
from collections import deque
from typing import Any, Protocol

TRAIL_MAX = 200
TRAIL_MIN_STEP_M = 0.12          # a new trail point only after the robot moved this far: in-place spins and pose jitter add nothing
TRAIL_KEEP_M = 12.0              # the trail keeps the last N metres of path, older points are dropped


class TrailBuffer:
    """Decimated trail: points >= `min_step` apart (spinning on the spot and localisation jitter are skipped), at most `keep_m` of path length."""

    def __init__(self, min_step: float = TRAIL_MIN_STEP_M, keep_m: float = TRAIL_KEEP_M, maxlen: int = TRAIL_MAX):
        self.min_step, self.keep_m = min_step, keep_m
        self._pts: deque[tuple[float, float]] = deque(maxlen=maxlen)

    def add(self, x: float, y: float) -> None:
        if self._pts and math.hypot(x - self._pts[-1][0], y - self._pts[-1][1]) < self.min_step:
            return
        if self._pts and math.hypot(x - self._pts[-1][0], y - self._pts[-1][1]) > 3.0:      # teleport (new world / relocalisation): start over
            self._pts.clear()
        self._pts.append((x, y))
        total, i = 0.0, len(self._pts) - 1
        while i > 0 and total <= self.keep_m:
            total += math.hypot(self._pts[i][0] - self._pts[i - 1][0], self._pts[i][1] - self._pts[i - 1][1])
            i -= 1
        for _ in range(i):
            self._pts.popleft()

    def clear(self) -> None:
        self._pts.clear()

    def points(self) -> list[list[float]]:
        return [[round(x, 3), round(y, 3)] for x, y in self._pts]

    def __len__(self) -> int:
        return len(self._pts)


def moves_of(backend) -> list[dict]:
    """Objects the incident graph saw MOVE (old -> new position), for the map arrows. [] when the backend keeps no timeline."""
    fn = getattr(backend, "moved_objects", None)
    try:
        return fn()[-8:] if fn else []
    except Exception:                                  # noqa: BLE001  a timeline problem must never break the state stream
        return []


class Telemetry(Protocol):
    def snapshot(self) -> dict[str, Any]: ...
    def map(self) -> dict[str, Any]: ...


def _rect(r) -> dict:
    return {"x0": r.x0, "y0": r.y0, "x1": r.x1, "y1": r.y1}


class MockTelemetry:
    """Reads `host.agent.layer.backend` (a MockBackend). The backend is not thread-safe: reads happen under `host.lock` (the same lock
    `AgentHost.say` holds during an LLM turn). If the lock is busy longer than `lock_timeout_s` the LAST snapshot is returned with
    `stale: true`, so a slow turn never freezes the stream and nothing here can delay an e-stop (which does not use the lock)."""

    def __init__(self, host, lock_timeout_s: float = 0.2, wall=time.monotonic):
        self.host, self.lock_timeout_s, self.wall = host, lock_timeout_s, wall
        self._last: dict[str, Any] | None = None
        self._trail = TrailBuffer()

    def reset(self) -> None:
        """New world behind the host: forget the trail and the cached snapshot."""
        self._last = None
        self._trail.clear()

    @property
    def backend(self):
        return self.host.agent.layer.backend

    def snapshot(self) -> dict[str, Any]:
        lock = getattr(self.host, "lock", None)
        if lock is not None and not lock.acquire(timeout=self.lock_timeout_s):
            return {**(self._last or {"connected": True}), "stale": True}
        try:
            snap = self._read()
        finally:
            if lock is not None:
                lock.release()
        self._last = snap
        return snap

    def _read(self) -> dict[str, Any]:
        b = self.backend
        st, pos = b.status().data, b.where_am_i().data
        xy = (pos["x"], pos["y"])
        self._trail.add(*xy)
        objs = b.recall("object").data.get("records", [])
        return {
            "connected": True, "stale": False, "source": "mock", "sim_time_s": round(b.now(), 1),
            "pose": {"x": pos["x"], "y": pos["y"], "heading_deg": pos["heading_deg"], "place": pos["place"]},
            "mode": st["mode"], "moving": st["mode"] == "moving", "estop": st["estop"], "task": st["task"],
            "speed_mps": st["speed_mps"], "battery_pct": st["battery_pct"], "battery_simulated": st["battery_simulated"],
            "legs": st["legs"], "alerts": list(st["alerts_active"]),
            "objects": [{k: o[k] for k in ("id", "class", "color", "x", "y", "place", "age_s", "stale")} for o in objs],
            "zones": [dict(z) for z in getattr(b, "zones", [])],
            "trail": self._trail.points(),
            "path": self._planned_path(b),
            "moves": moves_of(b),
        }

    @staticmethod
    def _planned_path(b) -> list[list[float]]:
        """Remaining waypoints of the running task (robot position first), [] when idle."""
        t = getattr(b, "task", None)
        if t is None:
            return []
        pts = [[round(b.x, 2), round(b.y, 2)]]
        pts += [[round(s[1][0], 2), round(s[1][1], 2)] for s in t.segments if s and s[0] == "move"]
        return pts if len(pts) > 1 else []

    def map(self) -> dict[str, Any]:
        w = self.backend.world
        return {"map_id": w.map_id, "bounds": _rect(w.bounds),
                "rooms": {n: _rect(r) for n, r in w.rooms.items()},
                "walls": [_rect(r) for r in w.walls], "obstacles": [_rect(r) for r in w.obstacles],
                "places": {n: {"x": xy[0], "y": xy[1]} for n, xy in self.backend.places.items()}}
