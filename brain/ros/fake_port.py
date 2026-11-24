"""FakePort: a RosPort without ROS - fake messages and fake action servers for tests (no Gazebo, no rclpy).

Kinematics: a Nav2 goal drives the pose straight to the goal at `speed`; `send_cmd` is integrated like cmd_vel.
Scenario hooks: `block_goal`, `unreachable`, `reveal_after`, `alerts`, `set_objects`, `leg_states_value`.
"""
from __future__ import annotations

import json
import math
from typing import Any

from .port import NAV_ABORTED, NAV_CANCELED, NAV_OK, NAV_RUNNING


class FakePort:
    def __init__(self, pose=(0.0, 0.0, 0.0), speed: float = 0.1):
        self.t = 0.0
        self.x, self.y, self.yaw = pose
        self.cruise = speed
        self.cmd = (0.0, 0.0, 0.0)
        self.cmds: list[tuple] = []
        self.objects: list[dict] = []
        self.blocked: list[tuple[float, float, float]] = []     # (x, y, r) goals that abort
        self.unreachable: list[tuple[float, float, float]] = []   # goals compute_path rejects
        self.occupied: list[tuple[float, float, float]] = []
        self.unknown_cells: list[tuple[float, float, float]] = []
        self.sectors: list[dict] = []
        self.battery: float | None = None
        self.legs: dict | None = None
        self._alerts: list[dict] = []
        self.estop = False
        self.explore_ok = True
        self.explore_finish_after = 60.0                        # sim seconds until the fake explorer reports "done" (0 = nothing left to explore)
        self.nav_up = True
        self.mapped = 0.4
        self.reveal: list[tuple[float, dict]] = []                # (explore seconds, object json) shown while exploring
        self.goals: dict[int, dict] = {}
        self.nav_log: list[tuple] = []
        self.explores: dict[int, dict] = {}
        self._h = 0
        self.policy = "policy"
        self.tilt = 0.5
        self.plan_requests = 0
        self.zero_holds: list[tuple[float, float]] = []         # (time, seconds) of every hold_zero call
        self.cancelled_goals: list[int] = []
        self.hidden: list[dict] = []                            # objects the camera has NOT seen yet: revealed when inside the camera cone (see set_camera)
        self.cam_range, self.cam_half_fov = 3.5, math.radians(35)

    # ---- scenario
    def set_objects(self, objs: list[dict]) -> None:
        self.objects = [dict(o) for o in objs]

    def hide_objects(self, objs: list[dict], cam_range: float = 3.5, cam_half_fov_deg: float = 35.0) -> None:
        """Objects that only appear in `semantic_objects()` once the robot's camera cone (range, half field of view) covers them: a robot that does not
        turn never sees what is behind it."""
        self.hidden = [dict(o) for o in objs]
        self.cam_range, self.cam_half_fov = cam_range, math.radians(cam_half_fov_deg)

    def alert(self, typ: str, severity: str, text: str, **data) -> None:
        self._alerts.append({"stamp": self.t, "type": typ, "severity": severity, "text": text, "data": data})

    def advance(self, dt: float) -> None:
        end = self.t + dt
        while self.t < end - 1e-9:
            h = min(0.1, end - self.t)
            self.t += h
            self._step(h)

    def _near(self, lst, x, y) -> bool:
        return any(math.hypot(x - a, y - b) <= r for a, b, r in lst)

    def _step(self, h: float) -> None:
        vx, vy, wz = self.cmd
        if vx or vy or wz:
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self.x += (vx * c - vy * s) * h
            self.y += (vx * s + vy * c) * h
            self.yaw += wz * h
        for g in self.goals.values():
            if g["state"] != NAV_RUNNING:
                continue
            if g["blocked"] and self.t - g["t0"] > 2.0:
                g["state"] = NAV_ABORTED
                continue
            d = math.hypot(g["x"] - self.x, g["y"] - self.y)
            step = self.cruise * h
            if d <= step:
                self.x, self.y = g["x"], g["y"]
                if g["yaw"] is not None:
                    self.yaw = g["yaw"]
                g["state"] = NAV_OK
            elif not g["blocked"]:
                self.x += (g["x"] - self.x) / d * step
                self.y += (g["y"] - self.y) / d * step
                self.yaw = math.atan2(g["y"] - self.y, g["x"] - self.x)
        for o in list(self.hidden):
            d = math.hypot(o["x"] - self.x, o["y"] - self.y)
            diff = (math.atan2(o["y"] - self.y, o["x"] - self.x) - self.yaw + math.pi) % (2 * math.pi) - math.pi
            if d <= self.cam_range and abs(diff) <= self.cam_half_fov:
                self.objects.append(dict(o, last_seen=self.t, first_seen=self.t))
                self.hidden.remove(o)
        for e in self.explores.values():
            if e["state"] != NAV_RUNNING:
                continue
            e["elapsed"] += h
            for when, obj in list(self.reveal):
                if e["elapsed"] >= when:
                    self.objects.append(dict(obj, last_seen=self.t, first_seen=self.t))
                    self.reveal.remove((when, obj))
            self.mapped = min(1.0, self.mapped + 0.004 * h)
            if e["elapsed"] >= e["max_time_s"] or (not self.reveal and e["elapsed"] >= e["finish_after"]):
                e["state"] = NAV_OK

    # ---- RosPort
    def now(self) -> float:
        return self.t

    def pose(self):
        return (self.x, self.y, self.yaw)

    def speed(self):
        return math.hypot(*self.cmd[:2]) if any(self.cmd) else 0.0

    def tilt_deg(self):
        return self.tilt

    def policy_status(self):
        return self.policy

    def semantic_objects(self):
        return json.loads(json.dumps(self.objects))      # like the JSON topic: a fresh parse each time

    def map_id(self) -> str:
        return "fake_lab"

    def map_cell(self, x, y):
        if self._near(self.occupied, x, y):
            return "occupied"
        return "unknown" if self._near(self.unknown_cells, x, y) else "free"

    def mapped_fraction(self):
        return self.mapped

    def obstacle_sectors(self, radius_m):
        return [s for s in self.sectors if s["distance_m"] <= radius_m]

    def battery_pct(self):
        return self.battery

    def leg_states(self):
        return self.legs

    def drain_alerts(self):
        out, self._alerts = self._alerts, []
        return out

    def nav_ready(self):
        return self.nav_up

    costmap_up = True
    lifecycle = None            # node -> label; None = every node reports "active"

    def global_costmap_ready(self):
        return self.costmap_up

    def lifecycle_state(self, node):
        return "active" if self.lifecycle is None else self.lifecycle.get(node)

    def nav_send(self, x, y, yaw):
        self._h += 1
        self.nav_log.append((x, y, yaw))
        self.goals[self._h] = {"x": x, "y": y, "yaw": yaw, "state": NAV_RUNNING, "t0": self.t, "blocked": self._near(self.blocked, x, y)}
        return self._h

    def nav_state(self, h):
        return {"state": self.goals[h]["state"], "distance_remaining": math.hypot(self.goals[h]["x"] - self.x, self.goals[h]["y"] - self.y),
                "recoveries": 2 if self.goals[h]["state"] == NAV_ABORTED else 0}

    def nav_cancel(self, h):
        self.cancelled_goals.append(h)
        if self.goals[h]["state"] == NAV_RUNNING:
            self.goals[h]["state"] = NAV_CANCELED

    def compute_path(self, x, y, start=None):
        self.plan_requests += 1
        if self._near(self.unreachable, x, y):
            return None
        x0, y0 = (self.x, self.y) if start is None else start
        n = 10
        pts = [(x0 + (x - x0) * i / n, y0 + (y - y0) * i / n) for i in range(n + 1)]
        return {"length_m": math.hypot(x - x0, y - y0), "points": pts}

    def explore_available(self):
        return self.explore_ok

    def explore_start(self, max_time_s, area_rect):
        self._h += 1
        self.explores[self._h] = {"state": NAV_RUNNING, "elapsed": 0.0, "max_time_s": max_time_s, "finish_after": self.explore_finish_after, "area_rect": area_rect}
        return self._h

    def explore_state(self, h):
        return {"state": self.explores[h]["state"]}

    def explore_cancel(self, h):
        if self.explores[h]["state"] == NAV_RUNNING:
            self.explores[h]["state"] = NAV_CANCELED

    def send_cmd(self, vx, vy, wz):
        self.cmd = (vx, vy, wz)
        self.cmds.append((vx, vy, wz))

    def hold_zero(self, seconds):
        self.zero_holds.append((self.t, seconds))
        self.cmd = (0.0, 0.0, 0.0)

    def set_estop(self, active):
        self.estop = active
        if active:
            self.cmd = (0.0, 0.0, 0.0)
            for g in self.goals.values():
                if g["state"] == NAV_RUNNING:
                    g["state"] = NAV_CANCELED

    def estop_active(self):
        return self.estop
