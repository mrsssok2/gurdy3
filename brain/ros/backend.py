"""RosBackend: RobotBackend over ROS 2 through a `RosPort` (design 1.3 / 5.6). Same result shapes as MockBackend.

Time only moves in the ROS world; the backend is stepped by `tick()` (a node timer, or `poll_events()` which the
agent loop calls anyway). Tool -> ROS mapping and the gaps are listed in docs/results/phase5_ros_backend.md.
Facts that have no ROS source yet are reported as unknown (None / "unknown"), never invented.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path
from typing import Any, Callable

from ..agent import templates
from ..mock.backend import STALE_AFTER_S, _norm, _wrap
from ..tools.schema import validate
from ..tools.specs import TOOLS
from ..tools.why import select_why
from ..tools.types import Result, bearing_word, fail, ok
from .port import NAV_ABORTED, NAV_CANCELED, NAV_OK, RosPort
from .relmotion import RelativeMotion

WALK_TIMEOUT_FACTOR = 3.0
NO_PROGRESS_S = 8.0
SEEN_NOW_S = 1.5            # an object re-detected within this many seconds counts as "in view"
DEDUP_S = 3.0              # the same alert type from the monitor and from this backend within this window = one event
MOVED_M = 0.3               # semantic-map position change that patrol reports as "moved"


class _Task:
    def __init__(self, tid: int, tool: str, args: dict, steps: list, final: Callable[[str, "_Task"], dict],
                 t0: float, max_time_s: float | None = None, until: Callable[[], bool] | None = None):
        self.id, self.tool, self.args, self.steps, self.final = tid, tool, args, list(steps), final
        self.t0, self.max_time_s, self.until = t0, max_time_s, until
        self.cur: dict | None = None        # active step state
        self.paused = False
        self.travelled = 0.0
        self.last_xy: tuple[float, float] | None = None
        self.done_steps: list[dict] = []    # per-step outcomes (patrol)


class RosBackend:
    def __init__(self, port: RosPort, places_file: str | Path | None = None):
        self.p = port
        self.places_file = Path(places_file) if places_file else None
        self.places: dict[str, dict] = {}          # name -> {x, y, kind, rect?}
        self.zones: list[dict] = []
        self.zones_version = 0           # bumped when a zone is added / removed: running navigation steps re-check their path
        self.doors: dict[str, tuple] = {}            # doorway centres from the places file (optional; VLN detects them on /map otherwise)
        self._scanned: dict[str, float] = {}       # room -> time its full-circle scan ended without the search ending (sweep order)
        self.task: _Task | None = None
        self.next_task = 1
        self.events: list[dict] = []
        self._cursor = 0
        self.missions: list[dict] = []
        self.schedules: list[dict] = []
        self._sched_n = 0
        self.alerts: list[str] = []
        self._load_places()

    # ================================================================ infrastructure
    def now(self) -> float:
        return self.p.now()

    def known_places(self) -> list[str]:
        self._ensure_home()
        return list(self.places)

    def poll_events(self) -> list[dict]:
        self.tick()
        out = self.events[self._cursor:]
        self._cursor = len(self.events)
        return out

    def _emit(self, typ: str, severity: str, text: str, src: str = "ros_backend", **data: Any) -> dict:
        ev = {"stamp": round(self.now(), 2), "type": typ, "severity": severity, "source": src, "text": text, "data": data}
        self.events.append(ev)
        return ev

    def _recent(self, typ: str, window_s: float) -> bool:
        return any(e["type"] == typ and self.now() - e["stamp"] <= window_s for e in self.events[-20:])

    def _emit_once(self, typ: str, severity: str, text: str, src: str, **data: Any) -> None:
        """Like _emit, but not if the alert monitor (or this backend) reported the same type a moment ago."""
        if not self._recent(typ, DEDUP_S):
            self._emit(typ, severity, text, src=src, **data)

    # ---- temporary zones (not a tool): an executor's zones for ONE walk (VLN avoided rooms). Same enforcement as the
    # user's zones (goals, Nav2 path check + re-check via zones_version, straight walks), but: never written to the places
    # file, removed by tag at the end of the walk, all removed on e-stop, and expired by `ttl_s` in tick() in case the
    # executor never comes back (a crash of the process loses them anyway: memory only).
    def add_temp_zone(self, rect, tag: str, name: str, ttl_s: float = 1800.0) -> int:
        x0, y0, x1, y1 = (float(v) for v in rect)
        zid = 1 + max((z["id"] for z in self.zones), default=0)
        self.zones.append({"id": zid, "name": name, "x": round((x0 + x1) / 2, 2), "y": round((y0 + y1) / 2, 2),
                           "r": round(math.hypot(x1 - x0, y1 - y0) / 2, 2), "rect": [x0, y0, x1, y1],
                           "temporary": True, "tag": tag, "expires": self.now() + ttl_s})
        self.zones_version += 1
        return zid

    def remove_temp_zones(self, tag: str | None) -> int:
        """Remove the temporary zones of `tag` (None = all temporary zones)."""
        n = len(self.zones)
        self.zones = [z for z in self.zones if not (z.get("temporary") and (tag is None or z.get("tag") == tag))]
        if len(self.zones) != n:
            self.zones_version += 1
        return n - len(self.zones)

    def _expire_temp_zones(self) -> None:
        now = self.now()
        old = [z for z in self.zones if z.get("temporary") and z.get("expires", 1e18) < now]
        if old:
            self.zones = [z for z in self.zones if z not in old]
            self.zones_version += 1
            self._emit("zone_expired", "warning", f"Temporary no-go zone {old[0]['name']} expired and was removed.")

    def tick(self) -> None:
        """Advance task state machines, schedules and the alert topic. Call at ~2-10 Hz."""
        self._expire_temp_zones()
        for a in self.p.drain_alerts():
            if self._recent(a.get("type", "alert"), DEDUP_S):      # the backend already reported the same thing itself
                continue
            ev = {"stamp": round(a.get("stamp", self.now()), 2), "type": a.get("type", "alert"),
                  "severity": a.get("severity", "info"), "source": a.get("source", "alert_topic"),
                  "text": a.get("text", ""), "data": a.get("data", {})}
            self.events.append(ev)
            if ev["severity"] in ("warning", "critical") and ev["type"] not in self.alerts:
                self.alerts.append(ev["type"])
            if ev["type"] in ("fall", "estop") and self.task:
                self._finish(self.task, "fell" if ev["type"] == "fall" else "estop")
        if self.task is not None:
            self._run_task(self.task)
        self._fire_schedules()

    # ================================================================ places / zones file
    def _load_places(self) -> None:
        if not self.places_file or not self.places_file.exists():
            return
        import yaml
        d = yaml.safe_load(self.places_file.read_text()) or {}
        from ..agent.rooms import canon_place
        for name, v in ((canon_place(n), v) for n, v in (d.get("places") or {}).items()):      # "room b" -> "room B" (display / speech)
            if "rect" in v:
                x0, y0, x1, y1 = v["rect"]
                self.places[name] = {"x": (x0 + x1) / 2, "y": (y0 + y1) / 2, "kind": v.get("kind", "room"), "rect": [x0, y0, x1, y1]}
                if v.get("names"):                       # spoken room names ("kitchen"), used by VLN (brain/vln/world.room_names)
                    self.places[name]["names"] = [str(x) for x in v["names"]]
            else:
                self.places[name] = {"x": v["x"], "y": v["y"], "kind": v.get("kind", "waypoint")}
        self.zones = [z for z in (d.get("no_go_zones") or []) if not z.get("temporary")]
        self.doors = {k: (float(v[0]), float(v[1])) for k, v in (d.get("doors") or {}).items()}   # optional: doorway centres (VLN)

    def _save_places(self) -> None:
        if not self.places_file:
            return
        import yaml
        out = {"map_id": self.p.map_id(), "places": {}, "no_go_zones": [z for z in self.zones if not z.get("temporary")]}
        if getattr(self, "doors", None):
            out["doors"] = {k: [round(v[0], 3), round(v[1], 3)] for k, v in self.doors.items()}
        for n, v in self.places.items():
            out["places"][n] = ({"rect": v["rect"], "kind": v["kind"], **({"names": v["names"]} if v.get("names") else {})} if "rect" in v
                                else {"x": round(v["x"], 3), "y": round(v["y"], 3), "kind": v["kind"]})
        self.places_file.write_text(yaml.safe_dump(out, sort_keys=False))

    def _ensure_home(self) -> None:
        if "home" not in self.places and (pose := self.p.pose()) is not None:
            self.places["home"] = {"x": round(pose[0], 3), "y": round(pose[1], 3), "kind": "waypoint"}   # start pose
            self._save_places()

    def _place_at(self, x: float, y: float) -> str:
        best = None
        for n, v in self.places.items():
            if "rect" in v:
                x0, y0, x1, y1 = v["rect"]
                if x0 <= x <= x1 and y0 <= y <= y1:
                    return n
            elif math.hypot(x - v["x"], y - v["y"]) <= 0.8 and (best is None or math.hypot(x - v["x"], y - v["y"]) < best[0]):
                best = (math.hypot(x - v["x"], y - v["y"]), n)
        return best[1] if best else "unnamed area"

    # ================================================================ helpers
    def _pose(self):
        return self.p.pose()

    def _no_pose(self) -> Result:
        return fail("UNAVAILABLE", "The robot pose is not available (no map to base_link transform).")

    def _tracks(self) -> list[dict]:
        return [t for t in (self.p.semantic_objects() or [])]

    def _obj(self, tr: dict, pose=None) -> dict:
        x, y, yaw = pose or self._pose()
        b = math.degrees(_wrap(math.atan2(tr["y"] - y, tr["x"] - x) - yaw))
        return {"id": tr["id"], "class": tr["class"], "color": tr.get("color") or "", "x": round(tr["x"], 2),
                "y": round(tr["y"], 2), "distance_m": round(math.hypot(tr["x"] - x, tr["y"] - y), 2),
                "bearing_deg": round(b, 1), "bearing_word": bearing_word(b), "place": self._place_at(tr["x"], tr["y"]),
                "last_seen_age_s": round(self.now() - tr.get("last_seen", self.now()), 1), "score": tr.get("score"), "hits": tr.get("hits")}

    def _match(self, ref: dict, in_place: str | None = None) -> list[dict]:
        """Seen objects of that class / colour, nearest first. `in_place`: only those standing INSIDE that place ("find the cone in room D":
        a cone remembered in room A does not count)."""
        x, y, _ = self._pose() or (0, 0, 0)
        out = [t for t in self._tracks() if t["class"] == ref["class"]
               and (not ref.get("color") or (t.get("color") or "").lower() == ref["color"].lower())
               and (in_place is None or self._inside(in_place, t["x"], t["y"]))]
        return sorted(out, key=lambda t: math.hypot(t["x"] - x, t["y"] - y))

    def _inside(self, place: str, x: float, y: float) -> bool:
        v = self.places.get(place)
        if v is None:
            return False
        if "rect" in v:
            x0, y0, x1, y1 = v["rect"]
            return x0 <= x <= x1 and y0 <= y <= y1
        return math.hypot(x - v["x"], y - v["y"]) <= 1.5

    def _unique(self, ref=None, oid=None):
        if oid is not None:
            tr = next((t for t in self._tracks() if t["id"] == oid), None)
            return (tr, None) if tr else (None, fail("NOT_SEEN", f"No object with id {oid} is known."))
        m = self._match(ref)
        name = f"{ref.get('color') or ''} {ref['class'].replace('_', ' ')}".strip()
        if not m:
            return None, fail("NOT_SEEN", f"I have not seen a {name}.")
        if len(m) > 1:
            return None, fail("AMBIGUOUS", f"I know of {len(m)} objects matching {name}.", candidates=[self._obj(t) for t in m])
        return m[0], None

    @staticmethod
    def _zone_has(z: dict, x: float, y: float) -> bool:
        """A zone made from a room is the room's rectangle (a disc around the centre let the planner cut through the corners and doorways of the
        room: held-out lab4 `l4_nogo_c_route`); a zone around an object is a disc."""
        if z.get("rect"):
            x0, y0, x1, y1 = z["rect"]
            return x0 <= x <= x1 and y0 <= y <= y1
        return math.hypot(x - z["x"], y - z["y"]) <= z["r"]

    def _in_zone(self, x: float, y: float) -> bool:
        return any(self._zone_has(z, x, y) for z in self.zones)

    def _path_crosses_zone(self, path: dict) -> bool:
        return any(self._in_zone(px, py) for px, py in path.get("points", []))

    def _zone_ids(self, x: float, y: float) -> set:
        return {z["id"] for z in self.zones if self._zone_has(z, x, y)}

    def _walk_zone_hit(self, direction: str, distance_m: float) -> dict | None:
        """The straight walk from the current pose (5 cm samples) enters a no-go zone? Leaving the zone the robot already
        stands in (moving away from its centre) is allowed. -> the zone or None."""
        pose = self._pose()
        if pose is None or not self.zones:
            return None
        x0, y0, yaw = pose
        dx, dy = {"forward": (1, 0), "backward": (-1, 0), "left": (0, 1), "right": (0, -1)}[direction]
        ux, uy = dx * math.cos(yaw) - dy * math.sin(yaw), dx * math.sin(yaw) + dy * math.cos(yaw)
        inside0 = self._zone_ids(x0, y0)
        for k in range(1, int(distance_m / 0.05) + 2):
            d = min(k * 0.05, distance_m)
            px, py = x0 + ux * d, y0 + uy * d
            for z in self.zones:
                if self._zone_has(z, px, py):
                    if z["id"] in inside0:
                        qx, qy = x0 + ux * max(d - 0.05, 0), y0 + uy * max(d - 0.05, 0)
                        if self._zone_has(z, qx, qy) and (z.get("rect") or math.hypot(px - z["x"], py - z["y"]) >= math.hypot(qx - z["x"], qy - z["y"]) - 1e-9):
                            continue      # walking out of (or inside) the zone we are in
                    return z
        return None

    def _zone_violation(self, s: dict, pose) -> bool:
        """Runtime guard for walk / turn: the robot entered a zone it was not in when the step started (drift, slip)."""
        entered = self._zone_ids(pose[0], pose[1]) - s.setdefault("zones0", self._zone_ids(pose[0], pose[1]))
        return bool(entered)

    def _dbg(self, msg: str) -> None:
        """Navigation decisions to stderr when GURDY_DEBUG_NAV is set (probe runs; the agent log keeps them)."""
        if os.environ.get("GURDY_DEBUG_NAV"):
            print(f"[nav {self.now():7.1f}s] {msg}", file=sys.stderr, flush=True)

    def _resolve(self, place=None, object=None, object_id=None, point=None, standoff_m=0.6):
        """-> (goal_xy, face_xy|None, info, path) or a Result error. `path` is the Nav2 plan to the goal."""
        pose = self._pose()
        if pose is None:
            return self._no_pose()
        if place is not None:
            self._ensure_home()
            names = {_norm(p): p for p in self.places}
            if _norm(place) not in names:
                return fail("UNKNOWN_PLACE", f"I do not know a place called '{place}'.", known_places=list(self.places))
            real = names[_norm(place)]
            v = self.places[real]
            if v["kind"] == "hazard":
                return fail("REFUSED", f"{real} is marked as a hazard.")
            if self._in_zone(v["x"], v["y"]):            # the place itself is a no-go zone: never "pick another point in it"
                return fail("IN_NO_GO_ZONE", "The target or the way to it is inside a no-go zone.", target={"kind": "place", "place": real})
            cands = [(g, None) for g in self._place_goals(v)]
            info = {"kind": "place", "place": real, "x": round(v["x"], 2), "y": round(v["y"], 2)}
        elif point is not None:
            cands = [((point["x"], point["y"]), None)]
            info = {"kind": "point", "x": point["x"], "y": point["y"]}
        else:
            tr, err = self._unique(object, object_id)
            if err:
                return err
            ring = sorted(((tr["x"] + standoff_m * math.cos(math.radians(15 * k)), tr["y"] + standoff_m * math.sin(math.radians(15 * k)))
                           for k in range(24)), key=lambda g: math.hypot(g[0] - pose[0], g[1] - pose[1]))
            cands = [(g, (tr["x"], tr["y"])) for g in ring[:8]]     # 8 closest ring points (each costs one plan request)
            info = {"kind": "object", "object": self._obj(tr, pose)}
        zone_hit = False
        for goal, face in cands:
            if self._in_zone(*goal):
                zone_hit = True
                continue
            if self.p.map_cell(*goal) == "occupied":
                continue
            path = self.p.compute_path(*goal)
            if path is None:
                continue
            crosses = self._path_crosses_zone(path)
            self._dbg(f"resolve goal ({goal[0]:.2f},{goal[1]:.2f}) from ({pose[0]:.2f},{pose[1]:.2f}): path {path['length_m']:.1f} m, {len(path['points'])} pts, crosses zone: {crosses}")
            if crosses:
                zone_hit = True
                path = self._detour(goal, pose)           # the planner's way leads through a zone: another way round, through a room that is allowed
                self._dbg(f"detour: {None if path is None else (path['via'], round(path['length_m'], 1))}")
                if path is None:
                    continue
            return goal, face, info, path
        if zone_hit:
            return fail("IN_NO_GO_ZONE", "The target or the way to it is inside a no-go zone.", target=info)
        return fail("NO_PATH", "There is no path to the target.", target=info)

    def _detour(self, goal, pose) -> dict | None:
        """The direct plan crosses a no-go zone: look for an intermediate room centre such that robot -> via and via -> goal both avoid every zone
        (the world is a ring of rooms: A-B-D-C-A with C forbidden still reaches D through B). Returns a path dict with `via` waypoints, or None.
        At most 4 candidates (each costs two plan requests), nearest to the goal first."""
        cands = []
        for n, v in self.places.items():
            if v.get("kind") != "room" or self._in_zone(v["x"], v["y"]):
                continue
            via = self._place_goals(v)[0]
            if math.hypot(via[0] - goal[0], via[1] - goal[1]) < 0.5 or math.hypot(via[0] - pose[0], via[1] - pose[1]) < 0.5:
                continue
            cands.append((math.hypot(via[0] - goal[0], via[1] - goal[1]) + math.hypot(via[0] - pose[0], via[1] - pose[1]), via))
        for _, via in sorted(cands)[:4]:
            p1 = self.p.compute_path(*via)
            if p1 is None or self._path_crosses_zone(p1):
                continue
            p2 = self.p.compute_path(*goal, start=via)
            if p2 is None or self._path_crosses_zone(p2):
                continue
            return {"length_m": p1["length_m"] + p2["length_m"], "points": p1["points"] + p2["points"], "via": [via]}
        return None

    CLEAR_M = 0.9      # a goal needs this much free space around it: costmap inflation is 0.55 m and a goal squeezed between two objects makes MPPI give up

    def _clear(self, x: float, y: float) -> bool:
        if self.p.map_cell(x, y) == "occupied":
            return False
        if any(self.p.map_cell(x + 0.5 * math.cos(a), y + 0.5 * math.sin(a)) == "occupied" for a in (k * math.pi / 4 for k in range(8))):
            return False
        return all(math.hypot(t["x"] - x, t["y"] - y) >= self.CLEAR_M for t in self._tracks())

    def _place_goals(self, v: dict) -> list[tuple[float, float]]:
        """Goal candidates for a place. A point place: itself. A room: its centre if it has clear space around it, otherwise the nearest points
        of a 0.5 m grid inside the room that do (the centre of room B sat between a cylinder and a box: Nav2 reached the room and then gave up).
        At most 8 (each costs one plan request); never empty."""
        centre = (v["x"], v["y"])
        if "rect" not in v:
            return [centre]
        x0, y0, x1, y1 = v["rect"]
        pts = [(x0 + 0.5 + i * 0.5, y0 + 0.5 + j * 0.5) for i in range(int(max(0.0, x1 - x0 - 1.0) / 0.5) + 1)
               for j in range(int(max(0.0, y1 - y0 - 1.0) / 0.5) + 1)]
        ok_pts = sorted([centre] + pts, key=lambda g: math.hypot(g[0] - centre[0], g[1] - centre[1]))
        clear = [g for g in ok_pts if self._clear(*g)][:8]
        return clear or [centre]

    def _close_enough(self, name: str | None, goal, pose) -> tuple[bool, bool]:
        """(close, inside): inside the target room, or within 1.0 m of the goal, when Nav2 gives up = arrived for our purposes."""
        v = self.places.get(name) if name else None
        inside = bool(v and "rect" in v and v["rect"][0] <= pose[0] <= v["rect"][2] and v["rect"][1] <= pose[1] <= v["rect"][3])
        near = math.hypot(goal[0] - pose[0], goal[1] - pose[1]) <= 1.0
        return inside or near, inside

    # ================================================================ task engine
    def _nav_ok(self) -> Result | None:
        if self.p.estop_active():
            return fail("ESTOP", "The emergency stop is active.")
        if self.task is not None:
            return fail("BUSY", f"I am busy with {self.task.tool}.", task={"task_id": self.task.id, "tool": self.task.tool})
        return None

    def _start(self, tool: str, args: dict, steps: list, final, max_time_s=None, until=None, estimate=None,
               needs_nav=True) -> Result:
        if (e := self._nav_ok()) is not None:
            return e
        if needs_nav and not self.p.nav_ready():
            return fail("UNAVAILABLE", "Navigation (Nav2 navigate_to_pose) is not running.")
        if tool != "return_home" and any(v == "failed" for v in (self.p.leg_states() or {}).values()):
            return fail("REFUSED", "A leg has failed; I will only go home.")
        pose = self._pose()
        if pose is None:
            return self._no_pose()
        task = _Task(self.next_task, tool, args, steps, final, self.now(), max_time_s, until)
        task.last_xy = pose[:2]
        self.next_task += 1
        self.task = task
        self.missions.append({"id": task.id, "tool": tool, "args": args, "start": self.now(), "end": None, "state": "running",
                              "tracks0": {t["id"]: (t["x"], t["y"]) for t in self._tracks()}, "distance_m": 0.0})
        self._emit("task_started", "info", f"{tool} started.", task_id=task.id, tool=tool)
        est = estimate or self._estimate(steps, pose)
        if not steps and until is None:
            self._finish(task, "done")
        elif steps:
            self._run_task(task)                      # send the first goal now, not at the next tick
        return ok(task_id=task.id, state="started", estimate=est)

    def _estimate(self, steps: list, pose) -> dict:
        d = 0.0
        for s in steps:
            if s["kind"] == "nav":
                d += s["length_m"] if s.get("length_m") is not None else math.hypot(s["x"] - pose[0], s["y"] - pose[1])
        return {"distance_m": round(d, 2), "duration_s": round(d / 0.10, 1)}

    def _run_task(self, t: _Task) -> None:
        pose = self._pose()
        if pose is None:
            return
        if t.last_xy and (d := math.hypot(pose[0] - t.last_xy[0], pose[1] - t.last_xy[1])) > 0.005:
            t.travelled += d
            t.last_xy = pose[:2]
        if t.paused:
            return
        if t.max_time_s is not None and self.now() - t.t0 > t.max_time_s:
            return self._finish(t, "timeout")
        if t.until is not None and t.until():
            self._cancel_step(t)
            t.steps = [s for s in t.steps if s.get("after_until")]
            t.until = None
            t.cur = None
        if t.cur is None:
            if not t.steps:
                return self._finish(t, ("reached" if t.tool in ("go_to", "return_home") else "done") if t.until is None else "timeout")
            t.cur = {**t.steps.pop(0), "started": False, "t0": self.now(), "best": None, "progress_t": self.now()}
        s = t.cur
        {"nav": self._step_nav, "rel": self._step_rel, "explore": self._step_explore, "rotate": self._step_rotate,
         "wait": self._step_wait}[s["kind"]](t, s, pose)

    def _next(self, t: _Task, outcome: dict | None = None) -> None:
        if outcome is not None:
            t.done_steps.append(outcome)
        t.cur = None

    def _cancel_step(self, t: _Task) -> None:
        s = t.cur
        if not s or not s.get("started"):
            return
        if s["kind"] == "nav" and s.get("h") is not None:
            self.p.nav_cancel(s["h"])        # whatever state Nav2 reports: also ends the recovery behaviours of the goal
        elif s["kind"] == "explore" and s.get("h") is not None:
            self.p.explore_cancel(s["h"])
        elif s["kind"] in ("rel", "rotate"):
            self.p.send_cmd(0.0, 0.0, 0.0)

    def _nav_zone_guard(self, t: _Task, s: dict, pose) -> bool:
        """No-go zones are enforced for the WHOLE walk, not only when it starts (held-out lab4 `l4_nogo_c_route`: the plan was checked once, the robot
        then walked through the forbidden room). (1) a step whose path was never checked (sweeps, patrols) is planned and checked before it starts;
        (2) every tick: the robot must not be in a zone it was not in when the step started; (3) a zone added or removed while the step runs makes
        the rest of the path be checked again. A path through a zone is re-routed through an allowed room when there is one, otherwise the mission
        stops and says why. -> True when the step was replaced / the task finished (the caller returns)."""
        ids = self._zone_ids(pose[0], pose[1])
        if s.get("zv") is None:
            s["zv"], s["zones0"] = self.zones_version, ids
            if not s.get("checked") and not s["started"]:
                return self._recheck_nav(t, s, pose)
        if s["zv"] != self.zones_version:
            s["zv"], s["zones0"] = self.zones_version, ids            # standing in a zone that was just added is not "entering" it
            if s["started"]:
                return self._recheck_nav(t, s, pose)
        if ids - s["zones0"]:
            self._dbg(f"ZONE ENTERED at ({pose[0]:.2f},{pose[1]:.2f}) step ({s['x']:.2f},{s['y']:.2f}) name={s.get('name')} via={bool(s.get('via'))}")
            self._cancel_step(t)
            self._emit("path_blocked", "warning", "I stopped at the edge of a no-go zone.", src="zone_guard", task_id=t.id)
            self._finish(t, "no_go_zone")
            return True
        return False

    def _recheck_nav(self, t: _Task, s: dict, pose) -> bool:
        goal = (s["x"], s["y"])
        if self._in_zone(*goal):
            path, det = {"points": [goal]}, None
        else:
            path = self.p.compute_path(*goal)
            det = None
        if path is not None and not self._path_crosses_zone(path):
            s["checked"] = True
            if not s.get("started") and (self._waypoints(path)):          # zones active: walk the checked route in short legs
                self._cancel_step(t)
                fresh = {k: v for k, v in s.items() if k in ("kind", "x", "y", "yaw", "name", "optional", "length_m")}
                legs = self._nav_steps(goal, None, path, s.get("name"))[:-1]
                for v in legs:
                    v["optional"] = bool(s.get("optional"))
                t.steps[:0] = legs + [{**fresh, "checked": True}]
                t.cur = None
                return True
            return False
        if path is not None and not self._in_zone(*goal):
            det = self._detour(goal, pose)
        self._cancel_step(t)
        if det is not None:                                           # another way round: waypoints first, then this step again
            fresh = {k: v for k, v in s.items() if k in ("kind", "x", "y", "yaw", "name", "optional", "length_m")}
            t.steps[:0] = self._nav_steps(goal, None, det, s.get("name"))[:-1] + [{**fresh, "checked": True}]
            t.cur = None
            return True
        if s.get("optional") and not s.get("started"):                # a sweep / patrol stop inside or behind a zone: leave it out
            self._next(t, {"place": s.get("name"), "reached": False})
            return True
        self._emit("path_blocked", "warning", "The way there leads through a no-go zone.", src="zone_guard", task_id=t.id)
        self._finish(t, "no_go_zone")
        return True

    def _step_nav(self, t: _Task, s: dict, pose) -> None:
        if self._nav_zone_guard(t, s, pose):
            return
        if not s["started"]:
            s["h"] = self.p.nav_send(s["x"], s["y"], s.get("yaw"))
            self._dbg(f"nav_send ({s['x']:.2f},{s['y']:.2f}) name={s.get('name')} via={bool(s.get('via'))} checked={s.get('checked')} from ({pose[0]:.2f},{pose[1]:.2f})")
            s["started"] = True
            if s["h"] is None:
                return self._finish(t, "no_path")
            return
        st = self.p.nav_state(s["h"])
        s["recoveries"] = st.get("recoveries", 0)
        if st["state"] == NAV_OK:
            self._next(t, None if s.get("via") else {"place": s.get("name"), "reached": True})
        elif st["state"] == NAV_ABORTED and t.tool in ("go_to", "return_home", "inspect") and not s.get("optional") and \
                (ce := self._close_enough(s.get("name"), (s["x"], s["y"]), pose))[0]:
            t.close_enough, t.inside = True, ce[1]               # Nav2 gave up inside the room / within 1 m of the goal: that is arrival
            self.p.nav_cancel(s["h"])
            self._next(t, {"place": s.get("name"), "reached": True, "close": True})
        elif st["state"] == NAV_ABORTED:
            self._dbg(f"nav ABORTED at ({pose[0]:.2f},{pose[1]:.2f}) step ({s['x']:.2f},{s['y']:.2f}) recoveries={s.get('recoveries')}")
            self._emit_once("path_blocked", "warning", "The path is blocked.", "nav_monitor", task_id=t.id)
            if s.get("optional"):
                self._next(t, None if s.get("via") else {"place": s.get("name"), "reached": False})
            else:
                self._finish(t, "blocked")
        elif st["state"] == NAV_CANCELED:
            if not t.paused:
                self._finish(t, "cancelled")

    def _step_rel(self, t: _Task, s: dict, pose) -> None:
        if not s["started"]:
            s["ctl"] = RelativeMotion(s["motion"], s["direction"], s["amount"], pose)
            s["started"] = True
        ctl: RelativeMotion = s["ctl"]
        if self._zone_violation(s, pose):
            self._emit("path_blocked", "warning", "I stopped at the edge of a no-go zone.", src="zone_guard", task_id=t.id)
            return self._finish(t, "no_go_zone")
        vx, vy, wz = ctl.command(pose)
        if ctl.done:
            self.p.send_cmd(0.0, 0.0, 0.0)
            return self._next(t, ctl.result())
        self.p.send_cmd(vx, vy, wz)
        prog = ctl.moved if ctl.kind == "walk" else abs(ctl.turned)
        if s["best"] is None or prog > s["best"] + (0.02 if ctl.kind == "walk" else 0.03):
            s["best"], s["progress_t"] = prog, self.now()
        elif self.now() - s["progress_t"] > NO_PROGRESS_S:
            self._emit_once("path_blocked", "warning", "I am not making progress.", "motion_monitor", task_id=t.id)
            self._finish(t, "blocked")

    def _step_rotate(self, t: _Task, s: dict, pose) -> None:
        """Turn to face a world point after arriving (go_to an object)."""
        want = math.atan2(s["fy"] - pose[1], s["fx"] - pose[0])
        err = _wrap(want - pose[2])
        if abs(err) <= math.radians(5) or self.now() - s["t0"] > 25:
            self.p.send_cmd(0.0, 0.0, 0.0)
            return self._next(t)
        s["started"] = True
        self.p.send_cmd(0.0, 0.0, math.copysign(0.3 if abs(err) > 0.3 else 0.23, err))

    def _step_wait(self, t: _Task, s: dict, pose) -> None:
        """Stand still for s["secs"] sim seconds (the camera / detector needs a moment on each heading of a scan)."""
        if not s["started"]:
            s["started"], s["t_end"] = True, self.now() + s["secs"]
            self.p.send_cmd(0.0, 0.0, 0.0)
        if self.now() >= s["t_end"]:
            if s.get("mark_room"):
                self._scanned[s["mark_room"]] = self.now()
            self._next(t)

    # ---- looking around: a sweep visits rooms and turns a full circle in each (the camera sees only a cone ahead)
    SCAN_QUARTERS = 4           # headings per scan: 90 degrees apart
    SCAN_DWELL_S = 2.0          # stand still per heading, sim seconds (the detector confirms a track over >= 2 s: min_span; was 2.5)
    ROOMS_MAPPED_MIN = 0.85     # share of known map cells inside the rooms from which `find` sweeps the rooms before it explores
    EXPLORE_TRIVIAL_S = 30.0    # an explore step that ended within this time and moved less than 1 m found nothing left to explore

    def _scan_steps(self) -> list[dict]:
        steps: list[dict] = []
        for _ in range(self.SCAN_QUARTERS):
            steps += [{"kind": "wait", "secs": self.SCAN_DWELL_S},
                      {"kind": "rel", "motion": "turn", "direction": "left", "amount": 360.0 / self.SCAN_QUARTERS}]
        return steps

    # search budget sized from the map (2026-11-27 dry run: a fixed 300 s is too short for a whole-house sweep; with
    # search_view_radius_m 1.5 the fake sweep needs > 900 s)
    SEARCH_SPEED_MPS = 0.10       # nominal walking speed of the search legs (mock / Nav2 cruise)
    SEARCH_TURN_RADPS = 0.3       # scan turn rate (relmotion)
    SEARCH_PATH_FACTOR = 1.3      # planned paths are longer than straight lines (doors)

    def search_budget_s(self, factor: float = 1.5, extra_s: float = 60.0) -> float:
        """Time the room sweep of `find` needs from the current pose: legs between the sweep viewpoints (straight line x
        SEARCH_PATH_FACTOR at SEARCH_SPEED_MPS) + every scan (dwell + quarter turns), x `factor`, + `extra_s`. Uses the same step
        list as `find` (rooms, viewpoints with search_view_radius_m, no-go rooms skipped), so it grows with the map."""
        pose = self._pose()
        cur = (pose[0], pose[1]) if pose else (0.0, 0.0)
        travel = scan = 0.0
        for s in self._sweep_steps():
            if s["kind"] == "nav":
                travel += math.hypot(s["x"] - cur[0], s["y"] - cur[1]) * self.SEARCH_PATH_FACTOR / self.SEARCH_SPEED_MPS
                cur = (s["x"], s["y"])
            elif s["kind"] == "wait":
                scan += s.get("secs", 0.0)
            elif s["kind"] == "rel" and s.get("motion") == "turn":
                scan += math.radians(s["amount"]) / self.SEARCH_TURN_RADPS
        return round((travel + scan) * factor + extra_s, 1)

    def _rooms_mapped(self) -> bool:
        """The named rooms are (almost) all known in the map: there is nothing worth exploring, the robot should go and LOOK. Without this the
        explore step of `find` ran first and walked the frontier cells of a saved map for the whole search budget (lab4 v2: 214-392 s
        searches never reached the room sweep). Unknown map -> False (explore first, as before)."""
        rooms = [v for v in self.places.values() if v.get("kind") == "room" and "rect" in v]
        known = total = 0
        for v in rooms:
            x0, y0, x1, y1 = v["rect"]
            for i in range(int((x1 - x0) / 0.5)):
                for j in range(int((y1 - y0) / 0.5)):
                    c = self.p.map_cell(x0 + 0.25 + 0.5 * i, y0 + 0.25 + 0.5 * j)
                    if c is None:
                        return False
                    total += 1
                    known += c != "unknown"
        return total > 0 and known / total >= self.ROOMS_MAPPED_MIN

    def _sweep_order(self, items: list, pose) -> list:
        """Rooms in the order they are visited: rooms not yet looked at in this session first, then the ones already scanned (an object that had been
        seen there would be in memory, so they are the least likely); inside each group a nearest-neighbour chain from the robot (rooms are a ring:
        A, B, D, C, not A, C, B, D)."""
        todo = [kv for kv in items if kv[0] not in self._scanned] + [kv for kv in items if kv[0] in self._scanned]
        if pose is None:
            return todo
        out, cur = [], (pose[0], pose[1])
        for group in ([kv for kv in todo if kv[0] not in self._scanned], [kv for kv in todo if kv[0] in self._scanned]):
            group = list(group)
            while group:
                nxt = min(group, key=lambda kv: math.hypot(kv[1]["x"] - cur[0], kv[1]["y"] - cur[1]))
                group.remove(nxt)
                out.append(nxt)
                cur = (nxt[1]["x"], nxt[1]["y"])
        return out

    def _sweep_steps(self, only: str | None = None) -> list[dict]:
        """Go to every room (or the one named place) that is not inside a no-go zone and look all around there. Used by `find` (object not in
        memory) and by `explore` when the explorer has nothing left to explore (an already mapped area): the robot still goes and looks."""
        steps: list[dict] = []
        pose = self._pose()
        items = list(self.places.items())
        if only is None:
            items = self._sweep_order([kv for kv in items if kv[1].get("kind") == "room"], pose)
        for n, v in items:
            if only is not None:
                if _norm(n) != _norm(only):
                    continue
            elif v.get("kind") != "room":
                continue
            if v.get("kind") == "hazard" or self._in_zone(v["x"], v["y"]):
                continue
            for k, (vx, vy) in enumerate(self._room_viewpoints(v)):
                steps.append({"kind": "nav", "x": vx, "y": vy, "yaw": None, "name": n if k == 0 else f"{n} view {k + 1}",
                              "optional": True})
                steps += self._scan_steps()
            steps.append({"kind": "wait", "secs": 0.0, "mark_room": n})      # the whole circle was done without the search ending: room looked at
        return steps

    # Multi-viewpoint room sweep (2026-11-27): one viewpoint per room leaves room corners ~2.8 m away in lab4, where the small
    # pickable classes are hard to see (v3 labels drop far small objects). None = the old behaviour (the room point only).
    search_view_radius_m: float | None = None
    VIEWPOINT_CLEARANCE_M = 0.45                    # free map cells this far around a viewpoint (footprint 0.34 + margin)

    def _room_viewpoints(self, room: dict) -> list[tuple]:
        """The room point first; with `search_view_radius_m`, add grid points so that every point of the room rectangle is within
        that radius of some viewpoint (square cells of side radius * sqrt 2), keeping only points whose surroundings are free
        in the map. Unreachable ones are skipped at run time (optional nav steps)."""
        pts = [(room["x"], room["y"])]
        R = self.search_view_radius_m
        if not R or "rect" not in room:
            return pts
        x0, y0, x1, y1 = room["rect"]
        side = R * math.sqrt(2)
        nx, ny = max(1, math.ceil((x1 - x0) / side)), max(1, math.ceil((y1 - y0) / side))
        c = self.VIEWPOINT_CLEARANCE_M
        for i in range(nx):
            for j in range(ny):
                gx, gy = x0 + (i + 0.5) * (x1 - x0) / nx, y0 + (j + 0.5) * (y1 - y0) / ny
                gx, gy = min(max(gx, x0 + c), x1 - c), min(max(gy, y0 + c), y1 - c)
                if math.hypot(gx - room["x"], gy - room["y"]) < 0.5 * R:
                    continue                            # the room point already covers it
                ring = [(gx + c * math.cos(a), gy + c * math.sin(a)) for a in [k * math.pi / 4 for k in range(8)]] + [(gx, gy)]
                if all(self.p.map_cell(px, py) in ("free", None) for px, py in ring):
                    pts.append((round(gx, 3), round(gy, 3)))
        # nearest-neighbour order from the room point
        out, rest = [pts[0]], pts[1:]
        while rest:
            q = min(rest, key=lambda p: math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]))
            rest.remove(q)
            out.append(q)
        return out

    def _step_explore(self, t: _Task, s: dict, pose) -> None:
        if not s["started"]:
            s["h"] = self.p.explore_start(s.get("max_time_s") or 600.0, s.get("area"))
            s["started"], s["travel0"] = True, t.travelled
            if s["h"] is None:
                return self._finish(t, "blocked")
            return
        st = self.p.explore_state(s["h"])
        if st["state"] == NAV_OK:
            if s.get("sweep") is not None and self.now() - s["t0"] < self.EXPLORE_TRIVIAL_S and t.travelled - s["travel0"] < 1.0:
                t.steps[:0] = s["sweep"]                     # nothing left to explore: look around where the user asked instead of reporting "done"
            self._next(t)
        elif st["state"] == NAV_ABORTED:
            self._finish(t, "blocked")
        elif st["state"] == NAV_CANCELED and not t.paused:
            self._finish(t, "cancelled")

    def _finish(self, t: _Task, reason: str) -> None:
        self._cancel_step(t)
        self.p.send_cmd(0.0, 0.0, 0.0)
        self.p.hold_zero(2.0)          # a mission reported as stopped / failed must really stop (smoother / recovery commands may still arrive)
        self.task = None
        data = t.final(reason, t)
        data["stopped_reason"] = reason
        data["task_id"] = t.id
        failed = reason not in ("reached", "done")
        m = next(m for m in self.missions if m["id"] == t.id)
        m.update(end=self.now(), state=reason, distance_m=round(t.travelled, 2), result=data)
        self._emit("task_failed" if failed else "task_finished", "warning" if failed else "info",
                   (templates.failure_text(t.tool, reason, t.args, data) or "Stopped.") if failed else
                   (templates.describe_event({"type": "task_finished", "data": {**data, "tool": t.tool}}) or "Done."), tool=t.tool, **data)

    def _fire_schedules(self) -> None:
        for s in [s for s in self.schedules if s["fires_at"] <= self.now()]:
            self.schedules.remove(s)
            c = s["call"]
            problems = validate(c["args"], TOOLS[c["tool"]].parameters) if c["tool"] in TOOLS else ["unknown tool"]
            res = fail("INVALID_ARGS", "; ".join(problems)) if problems else getattr(self, c["tool"])(**c["args"])
            self._emit("schedule_fired", "info", f"Scheduled {c['tool']} ran.", schedule_id=s["id"], result=res.to_dict())

    # ---- final-result builders
    def _goto_final(self, info: dict, goal):
        def final(reason: str, t: _Task) -> dict:
            pose = self._pose() or (goal[0], goal[1], 0)
            d = {"reached": reason in ("reached", "done"), "target": info,
                 "final_distance_m": round(math.hypot(goal[0] - pose[0], goal[1] - pose[1]), 2),
                 "travelled_m": round(t.travelled, 2), "duration_s": round(self.now() - t.t0, 1)}
            if getattr(t, "close_enough", False):
                d.update(close_enough=True, inside=bool(getattr(t, "inside", False)))
            return d
        return final

    WAYPOINT_M = 2.0     # with a no-go zone active the walk is cut into legs this long (see _waypoints)

    def _waypoints(self, path: dict | None) -> list[tuple[float, float]]:
        """Points of the checked plan every WAYPOINT_M metres (not the last one). Nav2 re-plans a goal by itself all the time and knows nothing about
        our no-go zones: after a deviation or a recovery spin the shortest way from where it stands can lead through the forbidden room (held-out lab4
        v2 `nogo_c_route`: the robot ended at the C-A doorway in 2 of 3 runs). Short legs leave it no such way: each leg's shortest path stays on the
        checked route. Only used while a zone exists."""
        pts = (path or {}).get("points") or []
        if not self.zones or len(pts) < 2 or path.get("length_m", 0.0) < 1.5 * self.WAYPOINT_M:
            return []
        out, acc, last, end = [], 0.0, pts[0], pts[-1]
        for p in pts[1:]:
            acc += math.hypot(p[0] - last[0], p[1] - last[1])
            last = p
            if acc >= self.WAYPOINT_M and math.hypot(p[0] - end[0], p[1] - end[1]) >= 0.75 * self.WAYPOINT_M and not self._in_zone(*p) \
                    and self._open_space(*p):
                out.append((p[0], p[1]))
                acc = 0.0
        return out

    INFLATION_M = 0.55      # Nav2 inflation radius: a waypoint inside it (a doorway) is refused by the controller within a second

    def _open_space(self, x: float, y: float) -> bool:
        """No occupied map cell within the inflation radius: a leg may end here (doorways, wall corners and object gaps are not waypoints)."""
        if self.p.map_cell(x, y) == "occupied":
            return False
        return not any(self.p.map_cell(x + self.INFLATION_M * math.cos(a), y + self.INFLATION_M * math.sin(a)) == "occupied"
                       for a in (k * math.pi / 6 for k in range(12)))

    def _nav_steps(self, goal, face, path=None, name=None) -> list[dict]:
        legs = self._waypoints(path) or [tuple(v) for v in (path or {}).get("via", [])]      # waypoints first (a detour round a no-go zone, or legs)
        legged = bool(self._waypoints(path))       # legs share the plan's length: the leg steps count 0 m, the last step the whole way (estimate)
        steps = [{"kind": "nav", "x": v[0], "y": v[1], "yaw": None, "length_m": 0.0 if legged else None, "name": None, "via": True, "checked": True}
                 for v in legs]
        steps.append({"kind": "nav", "x": goal[0], "y": goal[1], "yaw": math.atan2(face[1] - goal[1], face[0] - goal[0]) if face else None,
                      "length_m": path["length_m"] if path else None, "name": name, "checked": path is not None})
        return steps

    # ================================================================ motion
    def walk(self, direction: str, distance_m: float) -> Result:
        ctl_est = RelativeMotion("walk", direction, distance_m, (0, 0, 0)).estimate
        if (z := self._walk_zone_hit(direction, distance_m)) is not None:
            return fail("IN_NO_GO_ZONE", f"Walking {direction} {distance_m:g} metres would enter the no-go zone {z['name']}.", zone=dict(z))
        def final(reason, t):
            c = t.done_steps[-1] if t.done_steps else (t.cur["ctl"].result() if t.cur and "ctl" in t.cur else {"moved_m": 0.0})
            return dict(c)
        return self._start("walk", {"direction": direction, "distance_m": distance_m},
                           [{"kind": "rel", "motion": "walk", "direction": direction, "amount": distance_m}], final,
                           max_time_s=distance_m / 0.10 * WALK_TIMEOUT_FACTOR + 10, estimate=ctl_est, needs_nav=False)

    def turn(self, direction: str, angle_deg: float) -> Result:
        ctl_est = RelativeMotion("turn", direction, angle_deg, (0, 0, 0)).estimate
        def final(reason, t):
            c = t.done_steps[-1] if t.done_steps else (t.cur["ctl"].result() if t.cur and "ctl" in t.cur else {"turned_deg": 0.0})
            return dict(c)
        return self._start("turn", {"direction": direction, "angle_deg": angle_deg},
                           [{"kind": "rel", "motion": "turn", "direction": direction, "amount": angle_deg}], final,
                           max_time_s=math.radians(angle_deg) / 0.3 * WALK_TIMEOUT_FACTOR + 10, estimate=ctl_est, needs_nav=False)

    def go_to(self, place=None, object=None, object_id=None, point=None, standoff_m: float = 0.6, _tool: str = "go_to") -> Result:
        r = self._resolve(place, object, object_id, point, standoff_m)
        if isinstance(r, Result):
            return r
        goal, face, info, path = r
        steps = self._nav_steps(goal, face, path, info.get("place"))
        args = {k: v for k, v in dict(place=place, object=object, object_id=object_id, point=point).items() if v is not None}
        return self._start(_tool, args, steps, self._goto_final(info, goal))

    def return_home(self) -> Result:
        return self.go_to(place="home", _tool="return_home")

    def _area_rect(self, area: str | None) -> list[float] | None:
        """Named place -> [x0, y0, x1, y1] in the map frame for the explorer (a point place = 4 m square around it)."""
        if area is None:
            return None
        v = next(v for n, v in self.places.items() if _norm(n) == _norm(area))
        return list(v["rect"]) if "rect" in v else [v["x"] - 2.0, v["y"] - 2.0, v["x"] + 2.0, v["y"] + 2.0]

    def explore(self, max_time_s: float = 600, area: str | None = None) -> Result:
        if not self.p.explore_available():
            return fail("UNAVAILABLE", "The exploration action server (/explore) is not running.")
        if area is not None and _norm(area) not in {_norm(n) for n in self.places}:
            return fail("UNKNOWN_PLACE", f"I do not know an area called '{area}'.", known_places=list(self.places))
        def final(reason, t):
            frac = self.p.mapped_fraction()
            before = next(m for m in self.missions if m["id"] == t.id)["tracks0"]
            return {"mapped_pct": None if frac is None else round(100 * frac, 1),
                    "new_objects": [self._obj(x) for x in self._tracks() if x["id"] not in before],
                    "travelled_m": round(t.travelled, 2), "duration_s": round(self.now() - t.t0, 1)}
        args = {"max_time_s": max_time_s, **({"area": area} if area else {})}
        return self._start("explore", args, [{"kind": "explore", "max_time_s": max_time_s, "area": self._area_rect(area), "sweep": self._sweep_steps(area)}],
                           final, max_time_s,
                           estimate={"distance_m": 0.0, "duration_s": float(max_time_s)})

    def find(self, object: dict, approach: bool = False, max_time_s: float = 300, in_place: str | None = None) -> Result:
        """`in_place` (set by the agent's routing, never by the planner): only an object INSIDE that place counts; if none is known there the
        robot goes to that place and looks (it does not explore the rest of the map)."""
        if in_place is not None:
            real = next((n for n in self.places if _norm(n) == _norm(in_place)), None)
            if real is None:
                return fail("UNKNOWN_PLACE", f"I do not know a place called '{in_place}'.", known_places=list(self.places))
            in_place = real
        known = self._match(object, in_place)
        source = "memory" if known else "live"
        t_search = self.now()
        def final(reason, t):
            m = self._match(object, in_place)
            d = {"found": bool(m) and reason != "timeout", "object": self._obj(m[0]) if m else None, "source": source,
                 "searched_s": round(self.now() - t_search, 1),
                 "areas_searched": sorted({self._place_at(*(self._pose() or (0, 0, 0))[:2])})}
            if in_place is not None:
                d["in_place"] = in_place
            return d
        args = {"object": object, "approach": approach, "max_time_s": max_time_s, **({"in_place": in_place} if in_place else {})}
        if known:
            if len(known) == 1 and approach:
                r = self._resolve(object_id=known[0]["id"]) if in_place else self._resolve(object=object)
                if isinstance(r, Result):
                    return r
                goal, face, info, path = r
                return self._start("find", args, self._nav_steps(goal, face, path) + [{"kind": "rotate", "fx": face[0], "fy": face[1]}],
                                   final, max_time_s)
            return self._start("find", args, [], final)
        if in_place is not None:
            steps = self._sweep_steps(only=in_place)
            if not steps:
                return fail("IN_NO_GO_ZONE" if self._in_zone(self.places[in_place]["x"], self.places[in_place]["y"]) else "NO_PATH",
                            f"I cannot go and look in {in_place}.")
            until = lambda: bool(self._match(object, in_place))
            return self._start("find", args, steps, final, max_time_s, until, estimate={"distance_m": 0.0, "duration_s": float(max_time_s)})
        if not self.p.explore_available():
            return fail("UNAVAILABLE", "The object is not in the map and the exploration action server (/explore) is not running.")
        # Rooms already in the map: look through them FIRST (nearest-neighbour chain, rooms not yet looked at first) and explore what is left
        # afterwards; unknown space: explore first, then sweep. `until` stops the plan when the object appears.
        sweep = self._sweep_steps()
        explore = [{"kind": "explore", "max_time_s": max_time_s, "area": None}]
        steps: list[dict] = sweep + explore if self._rooms_mapped() else explore + sweep
        until = lambda: bool(self._match(object))
        if approach:      # filled in once the object is seen: the step list is replaced when `until` fires
            def until():
                if not self._match(object):
                    return False
                r = self._resolve(object=object)
                if isinstance(r, Result):
                    return True
                goal, face, _, path = r
                t = self.task
                t.steps = self._nav_steps(goal, face, path) + [{"kind": "rotate", "fx": face[0], "fy": face[1], "after_until": True}]
                for s in t.steps:
                    s["after_until"] = True
                return True
        return self._start("find", args, steps, final, max_time_s, until, estimate={"distance_m": 0.0, "duration_s": float(max_time_s)})

    def patrol(self, places: list[str], loops: int = 1) -> Result:
        self._ensure_home()
        names = {_norm(p): p for p in self.places}
        seq = []
        for name in places:
            if _norm(name) not in names:
                return fail("UNKNOWN_PLACE", f"I do not know a place called '{name}'.", known_places=list(self.places))
            real = names[_norm(name)]
            if self.places[real]["kind"] == "hazard":
                return fail("REFUSED", f"{real} is marked as a hazard.")
            v = self.places[real]
            if self._in_zone(v["x"], v["y"]):
                return fail("IN_NO_GO_ZONE", f"{real} is inside a no-go zone.")
            seq.append((real, v))
        steps = [{"kind": "nav", "x": v["x"], "y": v["y"], "yaw": None, "name": n, "optional": True}
                 for _ in range(loops) for n, v in seq]
        snap0 = {x["id"]: (x["x"], x["y"], x) for x in self._tracks()}
        def final(reason, t):
            now_t = {x["id"]: x for x in self._tracks()}
            return {"places": list(t.done_steps), "reached": sum(1 for s in t.done_steps if s["reached"]),
                    "new": [self._obj(x) for i, x in now_t.items() if i not in snap0],
                    "missing": [self._obj(v[2]) for i, v in snap0.items() if i not in now_t],
                    "moved": [self._obj(x) for i, x in now_t.items() if i in snap0 and math.hypot(x["x"] - snap0[i][0], x["y"] - snap0[i][1]) > MOVED_M],
                    "travelled_m": round(t.travelled, 2), "duration_s": round(self.now() - t.t0, 1)}
        return self._start("patrol", {"places": places, "loops": loops}, steps, final)

    def inspect(self, object: dict | None = None, object_id: int | None = None) -> Result:
        tr, err = self._unique(object, object_id)
        if err:
            return err
        stored = (tr["x"], tr["y"])
        r = self._resolve(object_id=tr["id"])
        if isinstance(r, Result):
            return r
        goal, face, info, path = r
        def final(reason, t):
            cur = next((x for x in self._tracks() if x["id"] == tr["id"]), None)
            fresh = cur is not None and self.now() - cur.get("last_seen", 0) <= 5.0 and reason in ("reached", "done")
            return {"confirmed": bool(fresh), "object": self._obj(cur) if cur else None, "photo_path": None,
                    "moved_m": round(math.hypot(cur["x"] - stored[0], cur["y"] - stored[1]), 2) if cur else None}
        args = {"object": object} if object else {"object_id": object_id}
        return self._start("inspect", args, self._nav_steps(goal, face, path) + [{"kind": "rotate", "fx": face[0], "fy": face[1]}], final)

    def stop(self) -> Result:
        """Cancel the active task and zero the velocity at once (the always-available path; /estop is separate)."""
        t = self.task
        self.p.send_cmd(0.0, 0.0, 0.0)
        self.p.hold_zero(2.0)
        if t is None:
            self._user_stop(None)
            return ok(stopped=True, cancelled_task=None)
        tid, tool = t.id, t.tool
        self._finish(t, "cancelled")
        self._user_stop(tool)      # the cause "why did you stop" must name
        return ok(stopped=True, cancelled_task={"task_id": tid, "tool": tool})

    def _user_stop(self, tool) -> None:
        if not self._recent("user_stop", 1.0):          # the agent's e-stop fast path calls stop() twice (executor + tool)
            self._emit("user_stop", "info", "You told me to stop.", src="user", tool=tool)

    def estop(self, active: bool) -> Result:
        """Not a tool: the safety layer / e-stop button calls this. Latches /estop and cancels the task; temporary zones of
        the walk that was running (VLN) are removed too."""
        self.p.set_estop(active)
        if active and self.task:
            self._finish(self.task, "estop")
        if active:
            self.remove_temp_zones(None)
        self._emit("estop", "critical" if active else "info", "Emergency stop " + ("engaged." if active else "released."), src="safety")
        return ok(estop=active)

    def mission_control(self, action: str) -> Result:
        t = self.task
        if t is None:
            return fail("UNAVAILABLE", "There is no active mission.")
        ref = {"task_id": t.id, "tool": t.tool}
        if action == "pause":
            if not t.paused:
                self._cancel_step(t)                  # Nav2 has no pause: cancel the goal, re-send it on resume
                if t.cur is not None:
                    t.cur["started"] = False
                t.paused = True
                self._emit("task_paused", "info", f"{t.tool} paused.", task_id=t.id)
        elif action == "resume":
            if t.paused:
                t.paused = False
                self._emit("task_resumed", "info", f"{t.tool} resumed.", task_id=t.id)
        else:
            self._finish(t, "cancelled")
            return ok(state="cancelled", task=ref)
        return ok(state="paused" if t.paused else "running", task=ref)

    def schedule(self, delay_s: int, call: dict) -> Result:
        if call["tool"] not in TOOLS:
            return fail("UNKNOWN_TOOL", f"no tool named '{call['tool']}'")
        problems = validate(call["args"], TOOLS[call["tool"]].parameters)
        if problems:
            return fail("INVALID_ARGS", "; ".join(problems), problems=problems)
        self._sched_n += 1
        self.schedules.append({"id": self._sched_n, "fires_at": self.now() + delay_s, "call": call})
        return ok(schedule_id=self._sched_n, fires_at=round(self.now() + delay_s, 1))

    # ================================================================ questions
    def where_is(self, object: dict) -> Result:
        if self._pose() is None:
            return self._no_pose()
        m = self._match(object)
        if not m:
            return fail("NOT_SEEN", f"I have not seen a {object.get('color') or ''} {object['class'].replace('_', ' ')}.".replace("a  ", "a "))
        return ok(matches=[self._obj(t) for t in m])

    def whats_nearby(self, radius_m: float = 2.0) -> Result:
        pose = self._pose()
        if pose is None:
            return self._no_pose()
        objs = sorted((self._obj(t, pose) for t in self._tracks()), key=lambda o: o["distance_m"])
        objs = [o for o in objs if o["distance_m"] <= radius_m]
        obstacles = [{"bearing_word": bearing_word(s["bearing_deg"]), "distance_m": round(s["distance_m"], 2)}
                     for s in self.p.obstacle_sectors(radius_m)]
        return ok(objects=objs, obstacles=obstacles, place=self._place_at(pose[0], pose[1]))

    def where_am_i(self) -> Result:
        pose = self._pose()
        if pose is None:
            return self._no_pose()
        self._ensure_home()
        h = self.places["home"]
        return ok(x=round(pose[0], 2), y=round(pose[1], 2), heading_deg=round(math.degrees(pose[2]), 1),
                  place=self._place_at(pose[0], pose[1]), home_distance_m=round(math.hypot(pose[0] - h["x"], pose[1] - h["y"]), 2),
                  localisation="ok")        # TODO: from slam_toolbox covariance / EKF (not available as a topic yet)

    def relation(self, a: dict, b: dict) -> Result:
        pose = self._pose()
        if pose is None:
            return self._no_pose()
        ta, err = self._unique(a)
        if err:
            return err
        tb, err = self._unique(b)
        if err:
            return err
        ang = math.degrees(_wrap(math.atan2(ta["y"] - tb["y"], ta["x"] - tb["x"]) - pose[2]))
        return ok(a_from_b=bearing_word(ang), distance_m=round(math.hypot(ta["x"] - tb["x"], ta["y"] - tb["y"]), 2),
                  same_place=self._place_at(ta["x"], ta["y"]) == self._place_at(tb["x"], tb["y"]),
                  a=self._obj(ta, pose), b=self._obj(tb, pose))

    def path_check(self, place=None, object=None, object_id=None, point=None) -> Result:
        pose = self._pose()
        if pose is None:
            return self._no_pose()
        r = self._resolve(place, object, object_id, point)
        if not isinstance(r, Result):
            goal, _, info, path = r
            return ok(reachable=True, path_length_m=round(path["length_m"], 2), eta_s=round(path["length_m"] / 0.10, 1),
                      blocked_by=None, leaves_mapped_area=self.p.map_cell(*goal) == "unknown", crosses_no_go=False, target=info)
        if r.code in ("NO_PATH", "IN_NO_GO_ZONE"):
            info = r.error.get("target") or {}
            goal = (info.get("x"), info.get("y"))
            leaves = goal[0] is not None and self.p.map_cell(*goal) == "unknown"
            return ok(reachable=False, path_length_m=None, eta_s=None,
                      blocked_by="no-go zone" if r.code == "IN_NO_GO_ZONE" else "unknown obstacle",
                      leaves_mapped_area=leaves, crosses_no_go=r.code == "IN_NO_GO_ZONE", target=info)
        return r

    def status(self) -> Result:
        t = self.task
        estop = self.p.estop_active()
        mode = "estop" if estop else ("paused" if t and t.paused else ("moving" if t else "idle"))
        legs = self.p.leg_states()
        batt = self.p.battery_pct()
        sp = self.p.speed()
        return ok(battery_pct=batt, battery_simulated=True, legs=dict(legs) if legs else {}, legs_known=legs is not None,
                  mode=mode, task={"task_id": t.id, "tool": t.tool} if t else None,
                  speed_mps=None if sp is None else round(sp, 2), uptime_s=round(self.now(), 1), estop=estop,
                  alerts_active=list(self.alerts), policy=self.p.policy_status(), tilt_deg=self.p.tilt_deg())

    def why(self, about: str) -> Result:
        types = {"stop": ("task_failed", "task_finished", "estop", "path_blocked", "fall", "user_stop"),
                 "last_task": ("task_started", "task_finished", "task_failed", "path_blocked"),
                 "alert": ("leg_failure", "battery_low", "battery_critical", "path_blocked", "object_missing", "stuck", "fall"),
                 "refusal": ("refusal",)}[about]
        evs = [e for e in self.events if e["type"] in types]
        evs = select_why(about, self.events, evs)
        return ok(events=[{k: e[k] for k in ("stamp", "type", "severity", "text")} for e in evs])

    def recall(self, what: str, query: dict | None = None) -> Result:
        query = query or {}
        if what == "object":
            pose = self._pose()
            if pose is None:
                return self._no_pose()
            out = []
            for t in self._tracks():
                if query.get("class") and t["class"] != query["class"]:
                    continue
                if query.get("color") and (t.get("color") or "").lower() != query["color"].lower():
                    continue
                age = self.now() - t.get("last_seen", self.now())
                if query.get("since_s") is not None and age > query["since_s"]:
                    continue
                out.append({**self._obj(t, pose), "age_s": round(age, 1), "stale": age > STALE_AFTER_S, "state": "present",
                            "map_id": self.p.map_id()})
            return ok(records=out)
        if what == "mission":
            return ok(records=[{"id": m["id"], "tool": m["tool"], "state": m["state"],
                                "age_s": round(self.now() - (m["end"] or m["start"]), 1)} for m in self.missions])
        self._ensure_home()
        return ok(records=[{"name": n, "x": round(v["x"], 2), "y": round(v["y"], 2), "kind": v["kind"], "map_id": self.p.map_id()}
                           for n, v in self.places.items()])

    def describe_view(self, question: str | None = None) -> Result:
        pose = self._pose()
        if pose is None:
            return self._no_pose()
        vis = [self._obj(t, pose) for t in self._tracks() if self.now() - t.get("last_seen", -1e9) <= SEEN_NOW_S]
        text = ("I can see " + ", ".join(f"a {o['color']} {o['class'].replace('_', ' ')} ({o['bearing_word']})".replace("a  ", "a ") for o in vis) + "."
                if vis else "I do not see any known objects right now.")
        return ok(text=text, unverified=True, detections=vis, photo_path=None)     # no VLM / photo yet: detector output only

    def report(self, mission_id: int | None = None) -> Result:
        done = [m for m in self.missions if m["end"] is not None]
        m = next((x for x in done if x["id"] == mission_id), None) if mission_id else (done[-1] if done else None)
        if m is None:
            return fail("UNAVAILABLE", "There is no finished mission to report.")
        evs = [e for e in self.events if m["start"] <= e["stamp"] <= m["end"]]
        new = [self._obj(t) for t in self._tracks() if t["id"] not in m["tracks0"]] if self._pose() else []
        return ok(summary={"mission_id": m["id"], "tool": m["tool"], "state": m["state"],
                           "duration_s": round(m["end"] - m["start"], 1), "distance_m": m["distance_m"], "objects": new,
                           "alerts": [e["text"] for e in evs if e["severity"] in ("warning", "critical")]}, report_path=None)

    # ================================================================ configuration
    def no_go_zone(self, action: str, place: str | None = None, around_object_id: int | None = None, radius_m: float = 0.8) -> Result:
        """Zones are enforced by this backend: goals, Nav2 plans, and straight walks (checked up front and while walking).
        Turning in place cannot enter a zone, so it is allowed. The Nav2 keepout costmap filter is still NEW (5.7)."""
        if action == "list":
            return ok(zones=[dict(z) for z in self.zones])
        if action == "add":
            if place is not None:
                self._ensure_home()
                names = {_norm(p): p for p in self.places}
                if _norm(place) not in names:
                    return fail("UNKNOWN_PLACE", f"I do not know a place called '{place}'.", known_places=list(self.places))
                v = self.places[names[_norm(place)]]
                x, y, name = v["x"], v["y"], names[_norm(place)]
                rect = v.get("rect")
            elif around_object_id is not None:
                tr = next((t for t in self._tracks() if t["id"] == around_object_id), None)
                if tr is None:
                    return fail("NOT_SEEN", f"No object with id {around_object_id} is known.")
                x, y, name = tr["x"], tr["y"], f"around #{around_object_id}"
                rect = None
            else:
                return fail("INVALID_ARGS", "give place or around_object_id")
            zone = {"id": 1 + max((z["id"] for z in self.zones), default=0), "name": name, "x": round(x, 2), "y": round(y, 2), "r": radius_m}
            if rect:
                zone["rect"] = [float(c) for c in rect]            # a room: the whole room (see _zone_has)
            self.zones.append(zone)
            self.zones_version += 1
            self._save_places()
            return ok(zones=[dict(z) for z in self.zones])
        keep = [z for z in self.zones if not ((place and _norm(z["name"]) == _norm(place))
                                              or (around_object_id and z["name"] == f"around #{around_object_id}"))]
        if len(keep) == len(self.zones):
            return fail("UNKNOWN_PLACE", "No such no-go zone.")
        self.zones = keep
        self.zones_version += 1
        self._save_places()
        return ok(zones=[dict(z) for z in self.zones])

    def save_place(self, name: str) -> Result:
        pose = self._pose()
        if pose is None:
            return self._no_pose()
        self._ensure_home()
        self.places[name] = {"x": round(pose[0], 2), "y": round(pose[1], 2), "kind": "waypoint"}
        self._save_places()
        return ok(place={"name": name, "x": round(pose[0], 2), "y": round(pose[1], 2)})
