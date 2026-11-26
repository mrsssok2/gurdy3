"""Mocked robot for fetch tasks (no ROS / Gazebo): the lab4_fetch world (rooms, small objects from its ground truth),
virtual time, and fault injection for every failure the state machine must handle.

faults: {"not_found": {names}, "unreachable": {names}, "grasp_fail": {name: n_failed_attempts}, "drop": {names},
         "slip": {names} (grasp closes but verify finds nothing), "no_path": {room names}, "user_takes": bool}
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GT = ROOT / "docs/results/data/lab4_fetch/lab4_fetch_gt.json"
ROOMS = {"room A": (-2.0, -2.0, 2.0, 2.0), "room B": (2.0, -2.0, 6.0, 2.0), "room C": (-2.0, 2.0, 2.0, 6.0), "room D": (2.0, 2.0, 6.0, 6.0)}
HOME = (0.0, 0.0)
WALK_SPEED = 0.10


def room_of(x, y):
    for n, (a, b, c, d) in ROOMS.items():
        if a <= x <= c and b <= y <= d:
            return n
    return None


@dataclass
class Obj:
    name: str
    cls: str
    color: str
    x: float
    y: float
    mass: float | None = None
    pickable: bool = False
    with_user: bool = False

    @property
    def room(self):
        return room_of(self.x, self.y)


@dataclass
class FetchMock:
    faults: dict = field(default_factory=dict)
    robot: tuple = HOME
    user: tuple = HOME                     # where the person who gives the commands is (the operator stands at home)
    t: float = 0.0
    carrying: str | None = None
    objects: dict = field(default_factory=dict)
    known: set = field(default_factory=set)     # objects already in the semantic map
    speeds: list = field(default_factory=list)  # every (speed, carrying) used for motion: carry-limit evidence
    grasp_attempts: dict = field(default_factory=dict)

    @classmethod
    def lab4_fetch(cls, faults: dict | None = None, all_known: bool = True, **kw):
        gt = json.loads(GT.read_text())
        m = cls(faults=faults or {}, **kw)
        for o in gt["objects"]:
            m.objects[o["name"]] = Obj(o["name"], o["class"], o.get("color", ""), o["x"], o["y"], o.get("mass"), o["pickable"])
        if all_known:
            m.known = set(m.objects)
        return m

    # ---- motion primitives (virtual time; speed recorded)
    def _move(self, xy, speed):
        d = math.hypot(xy[0] - self.robot[0], xy[1] - self.robot[1]) * 1.3          # paths are longer than straight lines
        self.speeds.append((speed, self.carrying is not None))
        self.t += d / speed
        self.robot = tuple(xy)

    def place_point(self, room: str):
        a, b, c, d = ROOMS[room]
        return ((a + c) / 2 + 0.6, (b + d) / 2 + 0.6)            # a free spot near the room centre (off the routes)

    def now(self):
        return self.t

    # ---- fetch primitives (each returns (ok, detail))
    def find(self, name):
        o = self.objects.get(name)
        if o is None or name in self.faults.get("not_found", set()):
            for r in ROOMS:                                        # searched every room
                self.t += 40.0
            return False, "not seen in any room"
        if name not in self.known:
            self.t += 60.0
            self.known.add(name)
        return True, o.room

    def approach(self, name, speed=WALK_SPEED):
        o = self.objects[name]
        if o.room in self.faults.get("no_path", set()):
            return False, f"no path to {o.room}"
        self._move((o.x + 0.3, o.y), speed)
        return True, o.room

    def fine_align(self, name):
        self.t += 8.0
        if name in self.faults.get("unreachable", set()):
            return False, 0.06                                     # residual error stays above 2 cm
        return True, 0.012

    def grasp(self, name):
        self.t += 6.0
        n = self.grasp_attempts.get(name, 0) + 1
        self.grasp_attempts[name] = n
        return n > self.faults.get("grasp_fail", {}).get(name, 0)

    def verify_grasp(self, name):
        self.t += 1.0
        if name in self.faults.get("slip", set()):
            return False
        self.carrying = name
        return True

    def fold_arm(self):
        self.t += 3.0
        return True

    def carry_to(self, xy, speed):
        if self.carrying and self.carrying in self.faults.get("drop", set()):
            mid = ((self.robot[0] + xy[0]) / 2, (self.robot[1] + xy[1]) / 2)
            self._move(mid, speed)
            o = self.objects[self.carrying]
            o.x, o.y = mid
            self.carrying = None
            return False, mid
        if room_of(*xy) in self.faults.get("no_path", set()):
            return False, None
        self._move(xy, speed)
        if self.carrying:
            o = self.objects[self.carrying]
            o.x, o.y = xy
        return True, xy

    def release(self):
        self.t += 3.0
        name, self.carrying = self.carrying, None
        return name is not None

    def hand_over(self, timeout_s=20.0):
        self.t += 4.0
        if not self.faults.get("user_takes", True):
            self.t += timeout_s
            return False
        self.carrying = None
        return True
