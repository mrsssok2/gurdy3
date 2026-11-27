"""What the VLN validator knows about the world (stdlib + yaml): rooms (rectangles), doorways (room graph), the semantic-map
objects (landmarks), the robot pose. Built from the places file + the world's door list + the semantic map JSON; for
tests / the DEV set from the lab4 ground truth (`VLNWorld.lab4()`)."""
from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]


@dataclass
class VLNWorld:
    rooms: dict                                  # "room A" -> (xmin, ymin, xmax, ymax)
    doors: dict                                  # "A-B" -> (x, y) doorway centre
    objects: list = field(default_factory=list)  # semantic map: [{id, class, color, x, y, ...}]
    robot: tuple = (0.0, 0.0, 0.0)               # x, y, yaw (map frame)

    @classmethod
    def lab4(cls, objects=None, robot=(0.0, 0.0, 0.0), drop=()):
        """lab4 rooms + doors; objects = the lab4 ground truth (all 10 big objects, as if all were seen) minus `drop` names."""
        places = yaml.safe_load((ROOT / "maps/lab4.places.yaml").read_text())["places"]
        rooms = {k: tuple(v["rect"]) for k, v in places.items() if v.get("kind") == "room"}
        gt = json.loads((ROOT / "docs/results/data/lab4/lab4_gt.json").read_text())
        doors = {k: tuple(v) for k, v in gt["doors"].items()}
        if objects is None:
            objects = [{"id": i + 1, "name": o["name"], "class": o["class"], "color": o.get("color") or "", "x": o["x"], "y": o["y"]}
                       for i, o in enumerate(gt["objects"]) if o["name"] not in drop]
        return cls(rooms, doors, list(objects), tuple(robot))

    @classmethod
    def from_backend(cls, be):
        """The world as the backend knows it NOW: rooms, doors, the semantic map (seen objects only), the robot pose.
        MockBackend: world.rooms / world.doors / tracks / x y yaw. Other backends: `places` rectangles + `doors` + `tracks()`
        (RosBackend: not wired yet)."""
        w = getattr(be, "world", None)
        if w is not None and hasattr(w, "rooms"):
            rooms = {n: (r.x0, r.y0, r.x1, r.y1) for n, r in w.rooms.items()}
            doors = dict(getattr(w, "doors", {}) or {})
            objs = [{"id": t["id"], "class": t["class"], "color": t.get("color") or "", "x": t["x"], "y": t["y"],
                     "last_seen": t.get("last_seen")} for t in be.tracks.values() if t.get("state", "present") == "present"]
            return cls(rooms, doors, objs, (be.x, be.y, be.yaw))
        raise NotImplementedError("VLNWorld.from_backend: only the mock backend so far")

    # ---- geometry
    def room_of(self, x, y, margin: float = 0.0):
        for n, (a, b, c, d) in self.rooms.items():
            if a + margin <= x <= c - margin and b + margin <= y <= d - margin:
                return n
        return None

    def centre(self, room):
        a, b, c, d = self.rooms[room]
        return ((a + c) / 2, (b + d) / 2)

    def door_rooms(self, door):
        a, b = door.split("-")
        return f"room {a}", f"room {b}"

    def neighbours(self, room):
        out = []
        for d in self.doors:
            r1, r2 = self.door_rooms(d)
            if room == r1:
                out.append((r2, d))
            elif room == r2:
                out.append((r1, d))
        return out

    def room_route(self, start, goal, avoid=()):
        """Shortest room sequence start -> goal through doorways, never entering an avoided room (start exempt).
        -> (rooms, doors) or None."""
        if start == goal:
            return [start], []
        prev = {start: None}
        q = deque([start])
        while q:
            r = q.popleft()
            for n, d in self.neighbours(r):
                if n in prev or (n in avoid and n != goal):
                    continue
                prev[n] = (r, d)
                if n == goal:
                    rooms, doors = [n], []
                    while prev[rooms[-1]] is not None:
                        p, dd = prev[rooms[-1]]
                        doors.append(dd)
                        rooms.append(p)
                    return rooms[::-1], doors[::-1]
                q.append(n)
        return None

    def room_routes(self, start, goal, avoid=()):
        """All simple room sequences start -> goal through doorways that never enter an avoided room (start exempt):
        [(rooms, doors)]. The lab maps are small (4 rooms), so enumerating is cheap."""
        out = []

        def dfs(r, rooms, doors):
            if r == goal:
                out.append((list(rooms), list(doors)))
                return
            for n, d in self.neighbours(r):
                if n in rooms or (n in avoid and n != goal):
                    continue
                dfs(n, rooms + [n], doors + [d])
        dfs(start, [start], [])
        return out

    def free(self, x, y, wall_margin: float = 0.35, obj_margin: float = 0.4, ignore=()) -> bool:
        if self.room_of(x, y, wall_margin) is None:
            return False
        return all(math.hypot(x - o["x"], y - o["y"]) >= obj_margin for o in self.objects if o.get("id") not in ignore)
