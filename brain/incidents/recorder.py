"""IncidentRecorder: turns what Gurdy already produces into graph events.

Inputs (all optional, all plain dicts, so the mock, the ROS backend and tests feed the same code):
  backend_events(evs)          backend `poll_events()` records (task_*, path_blocked, estop, user_stop, leg_failure, battery_*, object_missing ...)
  observe_objects(records, now_sim)   `recall("object")` records: id, class, color, x, y, place, age_s, state [, score]
  observe_pose(pose, now_sim)  `where_am_i` data: x, y, heading_deg, place -> room visited / left, "room checked" bookkeeping
  user_command(text, now_sim)  what the user said
  no_go(action, ...)           no-go zone set / cleared

Object identity (the hard part, all rules deterministic and conservative):
  * the same backend track id, or a same class + colour object within SAME_M, is the same object;
  * the same object seen again more than MOVE_M from where it was = `object_moved` (old -> new position, rooms);
  * `object_missing` = it was expected (last seen in room R) and the robot spent >= MIN_VISIT_S in R with the object's last position inside the
    camera range and field of view of some pose of that visit, without seeing it (or the backend itself said so after an `inspect`);
  * an object that was missing and a NEW same-class same-colour object that appeared later are ONE object that moved (merged, and the
    "new object" event is marked superseded); with two candidates nothing is merged (ambiguous: never guessed).
Times: every event has sim time (the robot's clock) and wall time (epoch seconds; `age_s` of a record is subtracted from both)."""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Any, Callable

from .store import IncidentStore

SAME_M = 0.6            # same class + colour within this distance = the same object
MOVE_M = 0.5            # the same object seen this far from its last position has moved (detector position noise is ~0.04 m)
SEEN_GAP_S = 60.0       # a re-sighting is logged only when the previous logged sighting is older than this (or the object had been missing / moved)
MIN_VISIT_S = 20.0      # the robot must stay this long in a room before an unseen object counts as missing
VIEW_M = 3.0            # an expected object counts as "looked at" when a pose of the visit was within this range ...
VIEW_HALF_DEG = 40.0    # ... and the object was within +- this angle of the heading (conservative: the camera is ~90 degrees wide)
CHECK_EVERY_S = 10.0    # while inside a room, re-run the missing check this often (sim seconds)
TICK_S = 0.5            # minimum sim time between two object / pose observations

ALERT_TYPES = {"leg_failure": "leg", "battery_low": "battery", "battery_critical": "battery", "fall": "fall", "stuck": "stuck"}
_ROOMISH = re.compile(r"^(room\b|lab\b|corridor|hall)", re.I)


def pretty(cls: str | None, color: str | None) -> str:
    return " ".join(p for p in ((color or "").strip(), (cls or "object").replace("_", " ")) if p)


@dataclass
class _Visit:
    room: str
    enter_sim: float
    enter_wall: float
    samples: list[tuple[float, float, float]]          # x, y, heading_deg
    last_check: float


class IncidentRecorder:
    def __init__(self, store: IncidentStore, is_room: Callable[[str], bool] | None = None):
        self.store = store
        self.is_room = is_room or (lambda name: bool(_ROOMISH.match(name or "")))
        self._objs: dict[int, dict] = {}              # node id -> {cls, color, x, y, room, last_sim, last_wall, state, first_wall, logged_wall}
        self._by_bid: dict[int, int] = {}             # backend track id (this session) -> object node id
        self._visit: _Visit | None = None
        self._last_tick = -1e9
        self._ev_cache: dict[str, int] = {}           # etype -> latest event id (for caused_by links)
        self._cmd_wall: float | None = None
        self._load()
        sid = store.add_event("session_start", "Recording started.", 0.0, store.wall())
        self._ev_cache["session_start"] = sid

    # ------------------------------------------------------------------ persistence
    def _load(self) -> None:
        for n in self.store.nodes("object"):
            d = n["data"]
            if d.get("merged_into"):
                continue
            self._objs[n["id"]] = {"cls": n["cls"], "color": n["color"] or "", "x": n["x"], "y": n["y"], "room": n["room"], "last_sim": -1e9,
                                   "last_wall": n["wall_t"] or 0.0, "state": n["state"] or "present", "first_wall": d.get("first_wall", n["wall_t"] or 0.0),
                                   "logged_wall": n["wall_t"] or 0.0, "first_event": d.get("first_event")}

    def _save(self, nid: int) -> None:
        o = self._objs[nid]
        self.store.update_node(nid, x=o["x"], y=o["y"], room=o["room"], sim_t=o["last_sim"], wall_t=o["last_wall"], state=o["state"],
                               data={"first_wall": o["first_wall"], "first_event": o.get("first_event")})

    # ------------------------------------------------------------------ helpers
    def _event(self, etype: str, text: str, sim_t: float, wall_t: float | None = None, severity: str = "info", cause: str | None = None,
               **kw: Any) -> int:
        eid = self.store.add_event(etype, text, sim_t, wall_t, severity, **kw)
        if cause and (cid := self._ev_cache.get(cause)):
            self.store.add_edge(eid, cid, "caused_by", sim_t, wall_t)
        self._ev_cache[etype] = eid
        return eid

    def _cause_within(self, etypes: tuple[str, ...], sim_t: float, within_s: float) -> int | None:
        best = None
        for et in etypes:
            eid = self._ev_cache.get(et)
            if eid is None:
                continue
            n = self.store.node(eid)
            if n and 0 <= sim_t - (n["sim_t"] or 0) <= within_s and (best is None or eid > best):
                best = eid
        return best

    def _link(self, eid: int, cause_id: int | None, sim_t: float, wall_t: float | None = None) -> None:
        if cause_id:
            self.store.add_edge(eid, cause_id, "caused_by", sim_t, wall_t)

    def _room_of(self, place: str | None) -> str | None:
        return place if place and self.is_room(place) else None

    # ------------------------------------------------------------------ backend events
    def backend_events(self, evs: list[dict], now_sim: float) -> None:
        wall_now = self.store.wall()
        for e in evs:
            t, d, sim = e["type"], e.get("data", {}) or {}, e.get("stamp", now_sim)
            wall = wall_now - max(0.0, now_sim - sim)              # the event's own wall time, not the time it was polled
            text = e.get("text", t)
            if t == "task_started":
                eid = self._event("task_started", text or f"Started {d.get('tool', 'a task')}.", sim, wall, data={"tool": d.get("tool"), "task_id": d.get("task_id")})
                if self._cmd_wall is not None and wall - self._cmd_wall <= 30:
                    self._link(eid, self._ev_cache.get("user_command"), sim, wall)
            elif t in ("task_finished", "task_failed"):
                reason = d.get("stopped_reason", "done" if t == "task_finished" else "failed")
                eid = self._event(t, text or f"{d.get('tool', 'The task')} {reason}.", sim, wall, "info" if t == "task_finished" else "warning",
                                  data={"tool": d.get("tool"), "reason": reason, "task_id": d.get("task_id"), "distance_m": d.get("distance_m")})
                if t == "task_failed":
                    self._link(eid, self._cause_within(("path_blocked", "alert"), sim, 30.0), sim, wall)
            elif t == "path_blocked":
                self._event("path_blocked", text or "My path is blocked.", sim, wall, "warning", room=self._visit.room if self._visit else None, data=dict(d))
            elif t in ("estop", "user_stop"):
                reason = "e-stop" if t == "estop" else "you told me to stop"
                eid = self._event("estop" if t == "estop" else "stop", text or f"Stopped: {reason}.", sim, wall, "critical" if t == "estop" else "info",
                                  data={"reason": reason, **d})
                cause = self._ev_cache.get("user_command") if self._cmd_wall is not None and wall - self._cmd_wall <= 15 else self._cause_within(("alert",), sim, 15.0)
                self._link(eid, cause, sim, wall)
            elif t in ALERT_TYPES:
                self._event("alert", text or t, sim, wall, e.get("severity", "warning"), data={"kind": ALERT_TYPES[t], "alert": t, **d})
            elif t == "object_missing":
                nid = self._by_bid.get(d.get("object_id"))
                if nid is not None:
                    self._mark_missing(nid, sim, wall, "reported by the robot's inspection")

    # ------------------------------------------------------------------ user + config
    def user_command(self, text: str, now_sim: float) -> None:
        wall = self.store.wall()
        eid = self._event("user_command", text.strip()[:200], now_sim, wall, data={"text": text.strip()[:200]})
        self.store.add_edge(eid, self.store.user_id, "issued_by", now_sim, wall)
        self._cmd_wall = wall

    def no_go(self, action: str, target: str, now_sim: float) -> None:
        if action == "add":
            self._event("no_go_set", f"No-go zone set: {target}.", now_sim, data={"target": target}, cause="user_command")
        elif action == "remove":
            self._event("no_go_cleared", f"No-go zone cleared: {target}.", now_sim, data={"target": target}, cause="user_command")

    # ------------------------------------------------------------------ pose -> room visits, missing check
    def observe_pose(self, pose: dict, now_sim: float) -> None:
        wall = self.store.wall()
        room = self._room_of(pose.get("place"))
        x, y, hdg = pose.get("x"), pose.get("y"), pose.get("heading_deg", 0.0)
        v = self._visit
        if v is not None and room != v.room:
            self._close_visit(now_sim, wall)
            v = self._visit = None
        if room is not None:
            if v is None:
                self._event("room_visited", f"Entered {room}.", now_sim, wall, room=room, x=x, y=y)
                v = self._visit = _Visit(room, now_sim, wall, [], now_sim)
            if x is not None:
                v.samples.append((x, y, hdg or 0.0))
            self.store.update_node(self.store.ensure_room(room), sim_t=now_sim, wall_t=wall)       # last time the robot was IN this room
            if now_sim - v.last_check >= CHECK_EVERY_S:
                v.last_check = now_sim
                self._check_missing(v, now_sim, wall)

    def _close_visit(self, now_sim: float, wall: float) -> None:
        v = self._visit
        self._check_missing(v, now_sim, wall)
        self._event("room_left", f"Left {v.room}.", now_sim, wall, room=v.room, data={"stayed_s": round(now_sim - v.enter_sim, 1)})
        self.store.update_node(self.store.ensure_room(v.room), sim_t=now_sim, wall_t=wall)

    def _covered(self, v: _Visit, ox: float, oy: float) -> bool:
        for x, y, hdg in v.samples:
            d = math.hypot(ox - x, oy - y)
            if d > VIEW_M:
                continue
            if d < 0.3:
                return True
            b = math.degrees(math.atan2(oy - y, ox - x)) - hdg
            b = (b + 180) % 360 - 180
            if abs(b) <= VIEW_HALF_DEG:
                return True
        return False

    def _check_missing(self, v: _Visit, now_sim: float, wall: float) -> None:
        if now_sim - v.enter_sim < MIN_VISIT_S:
            return
        for nid, o in list(self._objs.items()):
            if o["room"] != v.room or o["state"] != "present" or o["last_wall"] >= v.enter_wall:      # elsewhere / not expected / seen during this visit
                continue
            if self._covered(v, o["x"], o["y"]):
                self._mark_missing(nid, now_sim, wall, f"not seen in {v.room} during a {now_sim - v.enter_sim:.0f} s visit")

    def _mark_missing(self, nid: int, sim_t: float, wall: float, why: str) -> None:
        o = self._objs[nid]
        if o["state"] == "missing":
            return
        o["state"] = "missing"
        name = pretty(o["cls"], o["color"])
        self._event("object_missing", f"The {name} was not where I last saw it.", sim_t, wall, "warning", room=o["room"], x=o["x"], y=o["y"],
                    object_id=nid, cls=o["cls"], color=o["color"], data={"why": why, "last_seen_wall": o["last_wall"]})
        self._save(nid)
        self._try_merge_missing(nid, sim_t, wall)

    # ------------------------------------------------------------------ objects
    def observe_objects(self, records: list[dict], now_sim: float) -> None:
        wall = self.store.wall()
        for r in sorted(records, key=lambda r: r.get("age_s", 0.0), reverse=True):      # oldest sighting first = chronological
            if r.get("state") == "missing" or r.get("x") is None:
                continue
            age = float(r.get("age_s", r.get("last_seen_age_s", 0.0)) or 0.0)
            self._observe_one(r, now_sim - age, wall - age)

    def _observe_one(self, r: dict, sim_seen: float, wall_seen: float) -> None:
        bid, cls, color = r.get("id"), r["class"], (r.get("color") or "")
        x, y, room = r["x"], r["y"], self._room_of(r.get("place"))
        nid = self._by_bid.get(bid)
        if nid is None:
            nid = self._match(cls, color, x, y)
            if nid is not None:
                self._by_bid[bid] = nid
        if nid is None:
            return self._new_object(bid, cls, color, x, y, room, sim_seen, wall_seen, r.get("score"))
        o = self._objs[nid]
        if wall_seen <= o["last_wall"] + 1e-6:
            return                                                       # nothing newer than what we know (a stale track of a missing object too)
        moved_d = math.hypot(x - o["x"], y - o["y"])
        if moved_d > MOVE_M:
            self._moved(nid, x, y, room, sim_seen, wall_seen)
        else:
            returned = o["state"] == "missing"
            if returned or wall_seen - o["logged_wall"] >= SEEN_GAP_S or (room and room != o["room"]):
                self._seen(nid, x, y, room, sim_seen, wall_seen, r.get("score"), returned=returned)
            o.update(x=x if returned else o["x"], y=y if returned else o["y"], last_sim=sim_seen, last_wall=wall_seen, state="present")
            if room:
                o["room"] = room
            self._save(nid)

    def _match(self, cls: str, color: str, x: float, y: float) -> int | None:
        near = [(math.hypot(o["x"] - x, o["y"] - y), nid) for nid, o in self._objs.items()
                if o["cls"] == cls and o["color"] == color and o["state"] != "merged"]
        near = [t for t in near if t[0] <= SAME_M]
        if near:
            return min(near)[1]
        missing = [nid for nid, o in self._objs.items() if o["cls"] == cls and o["color"] == color and o["state"] == "missing"]
        return missing[0] if len(missing) == 1 else None                 # one missing candidate = it moved; two = ambiguous, never guessed

    def _new_object(self, bid, cls, color, x, y, room, sim_seen, wall_seen, score) -> None:
        name = pretty(cls, color)
        nid = self.store.add_node("object", key=f"{cls}|{color}|{len(self._objs) + 1}", label=name, cls=cls, color=color, room=room, x=x, y=y,
                                  sim_t=sim_seen, wall_t=wall_seen, state="present", data={"first_wall": wall_seen})
        self._objs[nid] = {"cls": cls, "color": color, "x": x, "y": y, "room": room, "last_sim": sim_seen, "last_wall": wall_seen, "state": "present",
                           "first_wall": wall_seen, "logged_wall": wall_seen, "first_event": None}
        if bid is not None:
            self._by_bid[bid] = nid
        eid = self._seen(nid, x, y, room, sim_seen, wall_seen, score, first=True)
        self._objs[nid]["first_event"] = eid
        self._save(nid)
        self._try_merge_new(nid, sim_seen, wall_seen)

    def _seen(self, nid, x, y, room, sim_seen, wall_seen, score, first=False, returned=False) -> int:
        o = self._objs[nid]
        name = pretty(o["cls"], o["color"])
        text = (f"Saw a new {name}" if first else f"The {name} is back" if returned else f"Saw the {name}") + (f" in {room}." if room else ".")
        eid = self.store.add_event("object_seen", text, sim_seen, wall_seen, room=room, x=x, y=y, object_id=nid, cls=o["cls"], color=o["color"],
                                   data={"confidence": score, "first": first, "returned": returned})
        if room:
            self.store.add_edge(nid, self.store.ensure_room(room), "seen_in", sim_seen, wall_seen)
        o["logged_wall"] = wall_seen
        self._ev_cache["object_seen"] = eid
        return eid

    def _moved(self, nid, x, y, room, sim_seen, wall_seen, merged_from: int | None = None) -> None:
        o = self._objs[nid]
        name = pretty(o["cls"], o["color"])
        fx, fy, froom = o["x"], o["y"], o["room"]
        where = f"from {froom} to {room}" if froom and room and froom != room else f"{math.hypot(x - fx, y - fy):.1f} m from where it was"
        eid = self.store.add_event("object_moved", f"The {name} moved {where}.", sim_seen, wall_seen, "warning", room=room, x=fx, y=fy, x2=x, y2=y,
                                   object_id=nid, cls=o["cls"], color=o["color"],
                                   data={"from": {"x": fx, "y": fy, "room": froom}, "to": {"x": x, "y": y, "room": room}, "distance_m": round(math.hypot(x - fx, y - fy), 2)})
        if froom:
            self.store.add_edge(eid, self.store.ensure_room(froom), "moved_from", sim_seen, wall_seen)
        if room:
            self.store.add_edge(eid, self.store.ensure_room(room), "moved_to", sim_seen, wall_seen)
            self.store.add_edge(nid, self.store.ensure_room(room), "seen_in", sim_seen, wall_seen)
        miss = self._ev_cache.get("object_missing")
        if o["state"] == "missing" and miss:
            self._link(eid, miss, sim_seen, wall_seen)
        self._ev_cache["object_moved"] = eid
        o.update(x=x, y=y, room=room or o["room"], last_sim=sim_seen, last_wall=wall_seen, state="present", logged_wall=wall_seen)
        self._save(nid)

    # ---- merging "missing here" + "new there" into one moved object
    def _try_merge_new(self, new_id: int, sim_t: float, wall: float) -> None:
        """A NEW object appeared while exactly one same class + colour object is missing: they are one object."""
        n = self._objs[new_id]
        cands = [i for i, o in self._objs.items() if i != new_id and o["cls"] == n["cls"] and o["color"] == n["color"] and o["state"] == "missing"]
        if len(cands) == 1:
            self._merge(cands[0], new_id, sim_t, wall)

    def _try_merge_missing(self, miss_id: int, sim_t: float, wall: float) -> None:
        """An object just went missing and exactly one same class + colour object was first seen AFTER its last sighting: it moved there."""
        m = self._objs[miss_id]
        cands = [i for i, o in self._objs.items() if i != miss_id and o["cls"] == m["cls"] and o["color"] == m["color"]
                 and o["state"] == "present" and o["first_wall"] > m["last_wall"]]
        if len(cands) == 1:
            self._merge(miss_id, cands[0], sim_t, wall)

    def _merge(self, keep: int, gone: int, sim_t: float, wall: float) -> None:
        g = self._objs.pop(gone)
        if g.get("first_event"):
            ev = self.store.node(g["first_event"])
            if ev:
                self.store.update_node(g["first_event"], data={**ev["data"], "superseded": True})
        self.store.update_node(gone, state="merged", data={"merged_into": keep, "first_wall": g["first_wall"]})
        for bid, nid in list(self._by_bid.items()):
            if nid == gone:
                self._by_bid[bid] = keep
        self.store.add_edge(gone, keep, "about", sim_t, wall, {"merged": True})
        self._moved(keep, g["x"], g["y"], g["room"], g["last_sim"], g["last_wall"])

    # ------------------------------------------------------------------ introspection (map arrows, web)
    def moved_objects(self, since_wall: float | None = None) -> list[dict]:
        return [{"object": pretty(e["cls"], e["color"]), "from": e["data"].get("from"), "to": e["data"].get("to"), "wall_t": e["wall_t"]}
                for e in self.store.events(since_wall=since_wall, etypes=["object_moved"])]
