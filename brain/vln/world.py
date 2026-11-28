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
    objects: list = field(default_factory=list)  # semantic map: [{id, class, color, x, y, sigma?, ...}] (sigma = xy 1-sigma, m)
    robot: tuple = (0.0, 0.0, 0.0)               # x, y, yaw (map frame)
    sigma_k: float = 3.0                         # clearance += sigma_k * position sigma (see `free`)
    margin: float = 0.0                          # extra fixed clearance (m) for every free-spot check
    names: dict = field(default_factory=dict)    # spoken room names -> room: {"kitchen": "room D"} (places file `names:`)
    places: tuple = ()                           # every place name the backend knows (rooms, waypoints, saved places)

    @classmethod
    def lab4(cls, objects=None, robot=(0.0, 0.0, 0.0), drop=(), names=None):
        """lab4 rooms + doors; objects = the lab4 ground truth (all 10 big objects, as if all were seen) minus `drop` names;
        names = extra spoken room names ({"kitchen": "room D"}), added to any in the places file."""
        places = yaml.safe_load((ROOT / "maps/lab4.places.yaml").read_text())["places"]
        rooms = {k: tuple(v["rect"]) for k, v in places.items() if v.get("kind") == "room"}
        nm = {**room_names(places), **{k.lower(): v for k, v in (names or {}).items()}}
        gt = json.loads((ROOT / "docs/results/data/lab4/lab4_gt.json").read_text())
        doors = {k: tuple(v) for k, v in gt["doors"].items()}
        if objects is None:
            objects = [{"id": i + 1, "name": o["name"], "class": o["class"], "color": o.get("color") or "", "x": o["x"], "y": o["y"]}
                       for i, o in enumerate(gt["objects"]) if o["name"] not in drop]
        return cls(rooms, doors, list(objects), tuple(robot), names=nm, places=tuple(places))

    @classmethod
    def from_backend(cls, be):
        """The world as the backend knows it NOW: rooms, doors, the semantic map (seen objects only), the robot pose.
        MockBackend: world.rooms / world.doors / tracks / x y yaw. Other backends: `places` rectangles + `doors` + `tracks()`
        (RosBackend: not wired yet)."""
        w = getattr(be, "world", None)
        if w is not None and hasattr(w, "rooms"):
            rooms = {n: (r.x0, r.y0, r.x1, r.y1) for n, r in w.rooms.items()}
            doors = dict(getattr(w, "doors", {}) or {})
            from ..mock.backend import OBS_SIGMA
            objs = [{"id": t["id"], "class": t["class"], "color": t.get("color") or "", "x": t["x"], "y": t["y"],
                     "last_seen": t.get("last_seen"), "sigma": OBS_SIGMA}         # the mock's sighting noise
                    for t in be.tracks.values() if t.get("state", "present") == "present"]
            nm = {k.lower(): v for k, v in (getattr(w, "room_names", None) or {}).items()}
            return cls(rooms, doors, objs, (be.x, be.y, be.yaw), names=nm, places=_known_places(be))
        if hasattr(be, "p") and hasattr(be, "places"):
            return cls.from_ros(be)
        raise NotImplementedError("VLNWorld.from_backend: mock or RosBackend")

    @classmethod
    def from_ros(cls, be, default_sigma: float = 0.10):
        """RosBackend: rooms = the places file's rectangles (kind room), doors = the places file's `doors` if given, else
        detected on the occupancy map (`doors_from_map`, cached per map), objects = the CONFIRMED semantic-map tracks with
        their position sigma (object3d `sigma_xyz`, else Phase 4 `sigma_m`, else `default_sigma`), robot = TF map -> base_link."""
        rooms = {n: tuple(float(c) for c in v["rect"]) for n, v in be.places.items() if v.get("rect") and v.get("kind", "room") == "room"}
        doors = dict(getattr(be, "doors", None) or {})
        if not doors:
            key = (be.p.map_id(), tuple(sorted(rooms.items())))
            cache = getattr(be, "_vln_doors", None)
            if not cache or cache[0] != key:
                be._vln_doors = (key, doors_from_map(rooms, be.p.map_cell))
            doors = dict(be._vln_doors[1])
        objs = []
        for t in be.p.semantic_objects() or []:
            sx = t.get("sigma_xyz")
            sig = max(sx[0], sx[1]) if sx else t.get("sigma_m", default_sigma)
            objs.append({"id": t["id"], "class": t["class"], "color": t.get("color") or "", "x": float(t["x"]), "y": float(t["y"]),
                         "last_seen": t.get("last_seen"), "sigma": float(sig if sig is not None else default_sigma)})
        pose = be.p.pose() or (0.0, 0.0, 0.0)
        return cls(rooms, doors, objs, tuple(pose), names=room_names(be.places), places=_known_places(be))

    # ---- room names
    def canon_room(self, name):
        """'room D' / 'D' / 'kitchen' / 'the kitchen' -> 'room D'; None if no such room."""
        if not isinstance(name, str):
            return None
        v = name.strip().lower()
        v = v[4:] if v.startswith("the ") else v
        for r in self.rooms:
            if r.lower() == v or r.lower() == f"room {v}":
                return r
        return self.names.get(v)

    def room_label(self, room) -> str:
        """'room D (the kitchen)' when the room has a spoken name, else 'room D'."""
        n = [k for k, v in self.names.items() if v == room]
        return f"{room} (the {n[0]})" if n else str(room)

    def rooms_text(self) -> str:
        return ", ".join(self.room_label(r) for r in sorted(self.rooms))

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

    def free(self, x, y, wall_margin: float = 0.35, obj_margin: float = 0.4, ignore=(), ref_sigma: float = 0.0) -> bool:
        """Is (x, y) a spot the robot fits? A point computed FROM a landmark (ref_sigma = that landmark's position sigma) is
        only as sure as the landmark: the wall clearance grows by sigma_k * ref_sigma, the clearance to another object by
        sigma_k * sqrt(sigma_obj^2 + ref_sigma^2), both plus `margin`. With sigmas the decision for a spot whose clearance is
        within the map's noise is a consistent NO instead of a coin flip (VLN DEV r01, vln_plan.md section 12)."""
        k, m = self.sigma_k, self.margin
        if self.room_of(x, y, wall_margin + m + k * ref_sigma) is None:
            return False
        return all(math.hypot(x - o["x"], y - o["y"]) >= obj_margin + m + k * math.hypot(o.get("sigma", 0.0), ref_sigma)
                   for o in self.objects if o.get("id") not in ignore)


def _known_places(be) -> tuple:
    try:
        return tuple(be.known_places())
    except Exception:                                   # noqa: BLE001
        return ()


def room_names(places: dict) -> dict:
    """Spoken names of the rooms from a places dict: `room D: {rect: [...], kind: room, names: [kitchen]}` -> {"kitchen": "room D"}."""
    out = {}
    for k, v in (places or {}).items():
        if isinstance(v, dict) and v.get("kind", "room") == "room" and v.get("rect"):
            for n in v.get("names") or []:
                out[str(n).strip().lower()] = k
    return out


def _letter(room: str) -> str:
    return room.split()[-1]


def doors_from_map(rooms: dict, map_cell, step: float = 0.05, side: float = 0.25, min_width: float = 0.5) -> dict:
    """Doorways between rectangular rooms, found on the occupancy map: for every pair of rooms that share an edge, walk along
    the shared edge; a sample is passable when the map is FREE on the edge and `side` m into both rooms. The longest
    passable run of at least `min_width` is the doorway (its centre). map_cell(x, y) -> "free" | "occupied" | "unknown" | None.
    -> {"A-B": (x, y)} (room names "room X"). Assumes axis-aligned rooms that touch (lab worlds)."""
    out = {}
    names = sorted(rooms)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            ax0, ay0, ax1, ay1 = rooms[a]
            bx0, by0, bx1, by1 = rooms[b]
            edge = None
            if abs(ax1 - bx0) < 0.06 or abs(bx1 - ax0) < 0.06:            # a vertical shared edge
                x = ax1 if abs(ax1 - bx0) < 0.06 else ax0
                lo, hi = max(ay0, by0), min(ay1, by1)
                edge = ("v", x, lo, hi)
            elif abs(ay1 - by0) < 0.06 or abs(by1 - ay0) < 0.06:          # a horizontal shared edge
                y = ay1 if abs(ay1 - by0) < 0.06 else ay0
                lo, hi = max(ax0, bx0), min(ax1, bx1)
                edge = ("h", y, lo, hi)
            if edge is None or edge[3] - edge[2] < min_width:
                continue
            kind, c, lo, hi = edge
            n = int((hi - lo) / step)
            run, best = [], []
            for k in range(n + 1):
                t = lo + k * step
                pts = [(c, t), (c - side, t), (c + side, t)] if kind == "v" else [(t, c), (t, c - side), (t, c + side)]
                if all(map_cell(px, py) == "free" for px, py in pts):
                    run.append(t)
                else:
                    best, run = (run if len(run) > len(best) else best), []
            best = run if len(run) > len(best) else best
            if best and best[-1] - best[0] >= min_width - step:
                mid = (best[0] + best[-1]) / 2
                pair = sorted([_letter(a), _letter(b)])
                out[f"{pair[0]}-{pair[1]}"] = (round(c, 3), round(mid, 3)) if kind == "v" else (round(mid, 3), round(c, 3))
    return out
