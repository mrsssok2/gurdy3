"""MockBackend: implements RobotBackend on a virtual clock (a 2-minute walk takes milliseconds of wall time).

What is simulated: kinematic motion at the nominal speeds, a camera cone with line of sight (objects become
*seen* with a fixed position error), room-level mapping, battery drain (a model, always labelled simulated),
no-go zones, events. NOT simulated: walking dynamics, Nav2 behaviour, detector errors, real timing
(design 5.3). Speeds are the Phase 1/3 command speeds, not measured Gazebo task times.

Time only moves in `advance(dt)` / `run_until_idle()`; tools never block.
"""
from __future__ import annotations

import math
import random
from typing import Any, Callable

from ..agent import templates
from ..tools.schema import validate
from ..tools.specs import TOOLS
from ..tools.why import select_why
from ..tools.types import Result, bearing_word, fail, ok
from .world import GtObject, World, flat_lab, zone_hit

V_WALK = 0.10        # m/s nominal command speed
W_TURN = 0.30        # rad/s
STEP = 0.25          # s, integration step
CAM_RANGE, CAM_FOV = 3.5, math.radians(90)   # assumed camera cone (ASSUMPTION, not the measured detector range)
OBS_SIGMA = 0.04     # m, position error of a seen object (Phase 4: ~0.03-0.04 m median)
STALE_AFTER_S = 1800.0
BATTERY_DRAIN_PCT_PER_S = 0.03   # simulated: 100 % lasts ~55 min of walking
LEGS = ("front_left", "middle_left", "rear_left", "front_right", "middle_right", "rear_right")
MOTION_TOOLS = ("walk", "turn", "go_to", "find", "explore", "patrol", "inspect", "return_home")


def _norm(name: str) -> str:
    n = name.lower().strip()
    for art in ("the ", "to ", "in "):
        if n.startswith(art):
            n = n[len(art):]
    return " ".join(n.split())


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class _Task:
    def __init__(self, tid: int, tool: str, args: dict, segments: list, final: Callable[[str, "_Task"], dict],
                 max_time_s: float | None = None, until: Callable[[], bool] | None = None):
        self.id, self.tool, self.args = tid, tool, args
        self.segments, self.final, self.until, self.max_time_s = list(segments), final, until, max_time_s
        self.t0 = 0.0
        self.travelled = 0.0
        self.turned = 0.0
        self.paused = False
        self.seg_state: dict = {}
        self.marks: list[dict] = []


class MockBackend:
    def __init__(self, world: World | None = None, seed: int = 0):
        self.world = world or flat_lab()
        self.rng = random.Random(seed)
        self.t = 0.0
        self.x, self.y, self.yaw = self.world.start
        self.battery = 100.0
        self.estop = False
        self.legs = {n: "ok" for n in LEGS}
        self.places: dict[str, tuple[float, float]] = dict(self.world.places)
        self.tracks: dict[int, dict] = {}          # track id -> record (the semantic map / memory)
        self.zones: list[dict] = []                # no-go circles
        self.task: _Task | None = None
        self.next_task = 1
        self.events: list[dict] = []
        self._cursor = 0
        self.missions: list[dict] = []
        self.schedules: list[dict] = []
        self.mapped_rooms = {self.world.place_at(self.x, self.y)}
        self.alerts: list[str] = []
        self.speed = 0.0
        self._update_seen()

    # ================================================================ infrastructure
    def now(self) -> float:
        return self.t

    def known_places(self) -> list[str]:
        return list(self.places)

    def poll_events(self) -> list[dict]:
        out = self.events[self._cursor:]
        self._cursor = len(self.events)
        return out

    def _emit(self, typ: str, severity: str, text: str, **data: Any) -> dict:
        ev = {"stamp": round(self.t, 2), "type": typ, "severity": severity, "source": "mock", "text": text,
              "data": data}
        self.events.append(ev)
        return ev

    # ---- test / scenario hooks (not tools)
    def advance(self, dt: float) -> None:
        end = self.t + dt
        while self.t < end - 1e-9:
            self._step(min(STEP, end - self.t))

    def run_until_idle(self, max_s: float = 3600.0) -> float:
        """Advance until the active task has finished (or max_s elapsed). Returns elapsed virtual seconds."""
        t0 = self.t
        while self.task is not None and self.t - t0 < max_s and not self.task.paused:
            self._step(STEP)
        return self.t - t0

    def add_obstacle(self, rect) -> None:
        self.world.obstacles.append(rect)
        self.world._build()

    def move_object(self, gid: int, x: float, y: float) -> None:
        o = self._gt(gid)
        o.x, o.y = x, y
        self.world._build()

    def remove_object(self, gid: int) -> None:
        self._gt(gid).present = False
        self.world._build()

    def fail_leg(self, leg: str) -> None:
        self.legs[leg] = "failed"
        self.alerts.append("leg_failure")
        self._emit("leg_failure", "critical", f"Leg {leg.replace('_', ' ')} has failed.", leg=leg)

    def _gt(self, gid: int) -> GtObject:
        return next(o for o in self.world.objects if o.gid == gid)

    # ================================================================ simulation core
    def _step(self, h: float) -> None:
        self.t += h
        self.speed = 0.0
        task = self.task
        if task is not None and not task.paused and not self.estop:
            self._run_task(task, h)
        self._update_seen()
        self._fire_schedules()

    def _update_seen(self) -> None:
        place = self.world.place_at(self.x, self.y)
        if place in self.world.rooms:
            self.mapped_rooms.add(place)
        for o in self.world.objects:
            if o.present and self._visible(o):
                self._sight(o)

    def _visible(self, o: GtObject) -> bool:
        d = math.hypot(o.x - self.x, o.y - self.y)
        if d > CAM_RANGE:
            return False
        b = _wrap(math.atan2(o.y - self.y, o.x - self.x) - self.yaw)
        return abs(b) <= CAM_FOV / 2 and self.world.line_of_sight((self.x, self.y), (o.x, o.y))

    def _sight(self, o: GtObject) -> None:
        tr = next((t for t in self.tracks.values() if t["gid"] == o.gid), None)
        if tr is None:
            tid = len(self.tracks) + 1
            tr = {"id": tid, "gid": o.gid, "class": o.cls, "color": o.color,
                  "x": o.x + self.rng.gauss(0, OBS_SIGMA), "y": o.y + self.rng.gauss(0, OBS_SIGMA),
                  "first_seen": self.t, "hits": 0, "score": round(self.rng.uniform(0.8, 0.95), 2), "state": "present"}
            self.tracks[tid] = tr
            self._emit("object_new", "info", f"New object: {' '.join(w for w in (o.color, o.cls.replace('_', ' ')) if w)}.", object_id=tid)
        elif math.hypot(tr["x"] - o.x, tr["y"] - o.y) > 0.5:       # the object really moved since it was last seen: the track follows (noise stays below 0.5 m)
            tr["x"], tr["y"] = o.x + self.rng.gauss(0, OBS_SIGMA), o.y + self.rng.gauss(0, OBS_SIGMA)
        tr["hits"] += 1
        tr["last_seen"] = self.t
        tr["state"] = "present"

    def _run_task(self, task: _Task, h: float) -> None:
        if task.max_time_s is not None and self.t - task.t0 > task.max_time_s:
            return self._finish(task, "timeout")
        if task.until is not None and task.until():
            return self._finish(task, "done")
        if not task.segments:
            return self._finish(task, "reached" if task.tool in ("go_to", "return_home") else "done")
        seg = task.segments[0]
        kind = seg[0]
        prev, self.battery = self.battery, max(0.0, self.battery - BATTERY_DRAIN_PCT_PER_S * h)
        for lim, typ, sev in ((25.0, "battery_low", "warning"), (10.0, "battery_critical", "critical")):
            if prev > lim >= self.battery:
                self._emit(typ, sev, f"Battery {typ.split('_')[1]}: {self.battery:.0f} percent (simulated).",
                           battery_pct=round(self.battery, 1), simulated=True)
        if self.battery <= 0:
            return self._finish(task, "timeout")
        if kind == "rotate_to" or kind == "rotate":
            if kind == "rotate_to":
                rem = _wrap(seg[1] - self.yaw)
            else:
                rem = seg[1]
            step = max(-W_TURN * h, min(W_TURN * h, rem))
            self.yaw = _wrap(self.yaw + step)
            task.turned += abs(step)
            if kind == "rotate":
                task.segments[0] = ("rotate", rem - step)
                rem = rem - step
            else:
                rem = _wrap(seg[1] - self.yaw)
            if abs(rem) < 1e-6:
                task.segments.pop(0)
        elif kind == "move":
            tx, ty = seg[1]
            dx, dy = tx - self.x, ty - self.y
            d = math.hypot(dx, dy)
            step = min(V_WALK * h, d)
            nx, ny = (self.x + dx / d * step, self.y + dy / d * step) if d > 1e-9 else (self.x, self.y)
            cur_blocked = not self.world.is_free(self.x, self.y)
            if step > 0 and not self.world.is_free(nx, ny) and not cur_blocked:
                return self._finish(task, "blocked")
            if step > 0 and any(zone_hit(t, nx, ny) for t in self._zone_circles()):
                return self._finish(task, "blocked")
            self.x, self.y = nx, ny
            task.travelled += step
            self.speed = V_WALK
            if d - step < 1e-6:
                task.segments.pop(0)
        elif kind == "mark":                           # patrol visit marker
            task.marks.append({"place": seg[1], "t": self.t, "tracks": set(self.tracks)})
            task.segments.pop(0)

    def _finish(self, task: _Task, reason: str) -> None:
        self.task = None
        self.speed = 0.0
        data = task.final(reason, task)
        if reason == "blocked" and task.tool in ("go_to", "return_home") and "final_distance_m" in data:
            place = (data.get("target") or {}).get("place")
            inside = bool(place and self.world.place_at(self.x, self.y) == place)
            if inside or data["final_distance_m"] <= 1.0:      # blocked inside the target room / within 1 m of the goal = arrived
                reason = "reached"
                data = task.final(reason, task)
                data.update(close_enough=True, inside=inside)
        data["stopped_reason"] = reason
        data["task_id"] = task.id
        failed = reason not in ("reached", "done")
        m = next(m for m in self.missions if m["id"] == task.id)
        m.update(end=self.t, state=reason, distance_m=round(task.travelled, 2), result=data)
        self._emit("task_failed" if failed else "task_finished", "warning" if failed else "info",
                   (templates.failure_text(task.tool, reason, task.args, data) or "Stopped.") if failed else
                   (templates.describe_event({"type": "task_finished", "data": {**data, "tool": task.tool}}) or "Done."), tool=task.tool, **data)
        if reason == "blocked":
            self._emit("path_blocked", "warning", "The path is blocked.", task_id=task.id)

    def _fire_schedules(self) -> None:
        due = [s for s in self.schedules if s["fires_at"] <= self.t]
        for s in due:
            self.schedules.remove(s)
            call = s["call"]
            problems = validate(call["args"], TOOLS[call["tool"]].parameters) if call["tool"] in TOOLS else ["unknown tool"]
            res = fail("INVALID_ARGS", "; ".join(problems)) if problems else getattr(self, call["tool"])(**call["args"])
            self._emit("schedule_fired", "info", f"Scheduled {call['tool']} ran.", schedule_id=s["id"], result=res.to_dict())

    # ================================================================ helpers
    def _obj(self, tr: dict) -> dict:
        d = math.hypot(tr["x"] - self.x, tr["y"] - self.y)
        b = math.degrees(_wrap(math.atan2(tr["y"] - self.y, tr["x"] - self.x) - self.yaw))
        return {"id": tr["id"], "class": tr["class"], "color": tr["color"], "x": round(tr["x"], 2),
                "y": round(tr["y"], 2), "distance_m": round(d, 2), "bearing_deg": round(b, 1),
                "bearing_word": bearing_word(b), "place": self.world.place_at(tr["x"], tr["y"]),
                "last_seen_age_s": round(self.t - tr["last_seen"], 1), "score": tr["score"]}

    def _inside(self, place: str, x: float, y: float) -> bool:
        r = next((r for n, r in self.world.rooms.items() if _norm(n) == _norm(place)), None)
        return r is not None and r.x0 <= x <= r.x1 and r.y0 <= y <= r.y1

    def _match(self, ref: dict, in_place: str | None = None) -> list[dict]:
        out = [t for t in self.tracks.values() if t["class"] == ref["class"]
               and (not ref.get("color") or t["color"].lower() == ref["color"].lower())
               and (in_place is None or self._inside(in_place, t["x"], t["y"]))]
        return sorted(out, key=lambda t: math.hypot(t["x"] - self.x, t["y"] - self.y))

    def _unique(self, ref: dict | None = None, oid: int | None = None) -> tuple[dict | None, Result | None]:
        if oid is not None:
            tr = self.tracks.get(oid)
            return (tr, None) if tr else (None, fail("NOT_SEEN", f"No object with id {oid} is known."))
        m = self._match(ref)
        name = f"{ref.get('color') or ''} {ref['class'].replace('_', ' ')}".strip()
        if not m:
            return None, fail("NOT_SEEN", f"I have not seen a {name}.")
        if len(m) > 1:
            return None, fail("AMBIGUOUS", f"I know of {len(m)} objects matching {name}.",
                              candidates=[self._obj(t) for t in m])
        return m[0], None

    def _in_zone(self, x: float, y: float) -> bool:
        return any(zone_hit(t, x, y) for t in self._zone_circles())

    def _zone_circles(self) -> list[tuple]:
        """Zones as tuples for the planner: circles (x, y, r); temporary room rectangles (x0, y0, x1, y1)."""
        return [tuple(z["rect"]) if "rect" in z else (z["x"], z["y"], z["r"]) for z in self.zones]

    # ---- temporary zones (not a tool: used by an executor, e.g. VLN avoided rooms, removed when it ends)
    def add_temp_zone(self, rect, tag: str, name: str) -> int:
        zid = 1 + max((z["id"] for z in self.zones), default=0)
        self.zones.append({"id": zid, "name": name, "rect": [float(v) for v in rect], "tag": tag, "temporary": True})
        return zid

    def remove_temp_zones(self, tag: str) -> int:
        n = len(self.zones)
        self.zones = [z for z in self.zones if z.get("tag") != tag]
        return n - len(self.zones)

    def _resolve(self, place=None, object=None, object_id=None, point=None, standoff_m=0.6):
        """-> (goal_xy, face_xy|None, target_info) or a Result error."""
        if place is not None:
            key = _norm(place)
            names = {_norm(p): p for p in self.places}
            if key not in names:
                return fail("UNKNOWN_PLACE", f"I do not know a place called '{place}'.", known_places=list(self.places))
            real = names[key]
            if real in self.world.hazards:
                return fail("REFUSED", f"{real} is marked as a hazard.")
            xy = self.places[real]
            return xy, None, {"kind": "place", "place": real, "x": xy[0], "y": xy[1]}
        if point is not None:
            return (point["x"], point["y"]), None, {"kind": "point", "x": point["x"], "y": point["y"]}
        tr, err = self._unique(object, object_id)
        if err:
            return err
        best = None
        for k in range(24):                      # nearest reachable ring point at the standoff distance
            a = math.radians(15 * k)
            g = (tr["x"] + standoff_m * math.cos(a), tr["y"] + standoff_m * math.sin(a))
            if not self.world.is_free(*g) or self._in_zone(*g):
                continue
            p = self.world.plan((self.x, self.y), g, self._zone_circles())
            if p is not None:
                L = self.world.path_length((self.x, self.y), p)
                if best is None or L < best[0]:
                    best = (L, g)
        info = {"kind": "object", "object": self._obj(tr)}
        if best is None:
            return fail("NO_PATH", "I cannot find a way to reach that object.", target=info)
        return best[1], (tr["x"], tr["y"]), info

    def _plan_to(self, goal: tuple[float, float], start: tuple[float, float] | None = None):
        s = start or (self.x, self.y)
        if self._in_zone(*goal):
            return fail("IN_NO_GO_ZONE", "The target is inside a no-go zone.")
        if not self.world.is_free(*goal):
            return fail("NO_PATH", "The target is in occupied or unknown space.")
        path = self.world.plan(s, goal, self._zone_circles())
        if path is None:
            if self.zones and self.world.plan(s, goal) is not None:
                return fail("IN_NO_GO_ZONE", "The only way there crosses a no-go zone.")
            return fail("NO_PATH", "There is no path to the target.")
        return path

    def _path_segments(self, start, path) -> list:
        segs, p = [], start
        for q in path:
            if math.hypot(q[0] - p[0], q[1] - p[1]) > 1e-6:
                segs += [("rotate_to", math.atan2(q[1] - p[1], q[0] - p[0])), ("move", q)]
            p = q
        return segs

    def _estimate(self, segs: list, start_yaw: float | None = None) -> dict:
        dist, rot, p, yaw = 0.0, 0.0, (self.x, self.y), self.yaw if start_yaw is None else start_yaw
        for s in segs:
            if s[0] == "move":
                dist += math.hypot(s[1][0] - p[0], s[1][1] - p[1])
                p = s[1]
            elif s[0] == "rotate_to":
                rot += abs(_wrap(s[1] - yaw))
                yaw = s[1]
            elif s[0] == "rotate":
                rot += abs(s[1])
                yaw = _wrap(yaw + s[1])
        return {"distance_m": round(dist, 2), "duration_s": round(dist / V_WALK + rot / W_TURN, 1)}

    def _start(self, tool: str, args: dict, segs: list, final, max_time_s=None, until=None,
               estimate: dict | None = None) -> Result:
        if self.estop:
            return fail("ESTOP", "The emergency stop is active.")
        if self.task is not None:
            return fail("BUSY", f"I am busy with {self.task.tool}.", task={"task_id": self.task.id, "tool": self.task.tool})
        if self.legs and any(v == "failed" for v in self.legs.values()) and tool != "return_home":
            return fail("REFUSED", "A leg has failed; I will only go home.")
        task = _Task(self.next_task, tool, args, segs, final, max_time_s, until)
        task.t0 = self.t
        self.next_task += 1
        self.task = task
        self.missions.append({"id": task.id, "tool": tool, "args": args, "start": self.t, "end": None, "state": "running",
                              "alerts": 0, "n_events0": len(self.events), "tracks0": set(self.tracks)})
        self._emit("task_started", "info", f"{tool} started.", task_id=task.id, tool=tool)
        est = estimate or self._estimate(segs)
        res = ok(task_id=task.id, state="started", estimate=est)
        if not segs and until is None:              # nothing to do: finish at once
            self._finish(task, "done")
        return res

    def _goto_final(self, target: dict, goal: tuple[float, float]):
        def final(reason: str, task: _Task) -> dict:
            return {"reached": reason in ("reached", "done"), "target": target,
                    "final_distance_m": round(math.hypot(goal[0] - self.x, goal[1] - self.y), 2),
                    "travelled_m": round(task.travelled, 2), "duration_s": round(self.t - task.t0, 1)}
        return final

    def _new_objects(self, task: _Task) -> list[dict]:
        before = next(m for m in self.missions if m["id"] == task.id)["tracks0"]
        return [self._obj(t) for i, t in self.tracks.items() if i not in before]

    # ================================================================ motion tools
    def walk(self, direction: str, distance_m: float) -> Result:
        ang = {"forward": 0.0, "left": math.pi / 2, "backward": math.pi, "right": -math.pi / 2}[direction]
        a = self.yaw + ang
        goal = (self.x + distance_m * math.cos(a), self.y + distance_m * math.sin(a))
        if self._in_zone(*goal):
            return fail("IN_NO_GO_ZONE", "That would take me into a no-go zone.")
        y0 = self.yaw

        def final(reason, task):
            return {"moved_m": round(task.travelled, 2)}
        return self._start("walk", {"direction": direction, "distance_m": distance_m}, [("move", goal)], final,
                           estimate=self._estimate([("move", goal)], y0))

    def turn(self, direction: str, angle_deg: float) -> Result:
        rad = math.radians(angle_deg) * (1 if direction == "left" else -1)

        def final(reason, task):
            return {"turned_deg": round(math.degrees(task.turned), 1)}
        return self._start("turn", {"direction": direction, "angle_deg": angle_deg}, [("rotate", rad)], final)

    def go_to(self, place=None, object=None, object_id=None, point=None, standoff_m: float = 0.6, _tool: str = "go_to") -> Result:
        r = self._resolve(place, object, object_id, point, standoff_m)
        if isinstance(r, Result):
            return r
        goal, face, info = r
        path = self._plan_to(goal)
        if isinstance(path, Result):
            return path
        segs = self._path_segments((self.x, self.y), path)
        if face:
            segs.append(("rotate_to", math.atan2(face[1] - goal[1], face[0] - goal[0])))
        args = {k: v for k, v in dict(place=place, object=object, object_id=object_id, point=point).items() if v is not None}
        return self._start(_tool, args, segs, self._goto_final(info, goal))

    def return_home(self) -> Result:
        return self.go_to(place="home", _tool="return_home")      # the tool name matters: a failed leg refuses everything but return_home

    def _viewpoints(self, area: str | None) -> list[tuple[float, float]]:
        """Scan points on a 2 m grid inside the rooms (free cells only), ordered nearest-neighbour from the robot."""
        pts = []
        for n, r in self.world.rooms.items():
            if area is not None and _norm(n) != _norm(area):
                continue
            nx, ny = max(1, round((r.x1 - r.x0) / 2.0)), max(1, round((r.y1 - r.y0) / 2.0))
            for i in range(nx):
                for j in range(ny):
                    p = (r.x0 + (i + 0.5) * (r.x1 - r.x0) / nx, r.y0 + (j + 0.5) * (r.y1 - r.y0) / ny)
                    if self.world.is_free(*p):
                        pts.append(p)
        order, cur = [], (self.x, self.y)
        while pts:
            nxt = min(pts, key=lambda q: math.hypot(q[0] - cur[0], q[1] - cur[1]))
            pts.remove(nxt)
            order.append(nxt)
            cur = nxt
        return order

    def _search_segments(self, area: str | None) -> list:
        segs, p = [], (self.x, self.y)
        for c in self._viewpoints(area):
            path = self.world.plan(p, c, self._zone_circles())
            if path is None:
                continue
            segs += self._path_segments(p, path) + [("rotate", 2 * math.pi)]
            p = c
        return segs

    def find(self, object: dict, approach: bool = False, max_time_s: float = 300, in_place: str | None = None) -> Result:
        name = f"{object.get('color') or ''} {object['class'].replace('_', ' ')}".strip()
        if in_place is not None:
            real = next((n for n in self.world.rooms if _norm(n) == _norm(in_place)), None)
            if real is None:
                return fail("UNKNOWN_PLACE", f"I do not know a place called '{in_place}'.", known_places=list(self.places))
            in_place = real
        known = self._match(object, in_place)
        source = "memory" if known else "live"
        searched0 = self.t

        def final(reason, task):
            m = self._match(object, in_place)
            return {"found": bool(m) and reason != "timeout", "object": self._obj(m[0]) if m else None,
                    "source": source, "searched_s": round(self.t - searched0, 1),
                    "areas_searched": sorted(self.mapped_rooms & set(self.world.rooms)), **({"in_place": in_place} if in_place else {})}
        args = {"object": object, "approach": approach, "max_time_s": max_time_s, **({"in_place": in_place} if in_place else {})}
        if known:
            if len(known) == 1 and approach:
                r = self._resolve(object_id=known[0]["id"]) if in_place else self._resolve(object=object)
                if isinstance(r, Result):
                    return r
                goal, face, info = r
                path = self._plan_to(goal)
                if isinstance(path, Result):
                    return path
                segs = self._path_segments((self.x, self.y), path) + [
                    ("rotate_to", math.atan2(face[1] - goal[1], face[0] - goal[0]))]
                return self._start("find", args, segs, final, max_time_s)
            return self._start("find", args, [], final)
        segs = self._search_segments(in_place)
        if not segs:
            return fail("NO_PATH", "I cannot reach any area to search.")
        def until():
            return bool(self._match(object, in_place))
        res = self._start("find", args, segs, final, max_time_s, until)
        return res

    def explore(self, max_time_s: float = 600, area: str | None = None) -> Result:
        if area is not None and _norm(area) not in {_norm(n) for n in self.world.rooms}:
            return fail("UNKNOWN_PLACE", f"I do not know an area called '{area}'.", known_places=list(self.places))
        segs = self._search_segments(area)

        def final(reason, task):
            total = sum(r.area for r in self.world.rooms.values())
            mapped = sum(r.area for n, r in self.world.rooms.items() if n in self.mapped_rooms)
            return {"mapped_pct": round(100 * mapped / total, 1), "new_objects": self._new_objects(task),
                    "travelled_m": round(task.travelled, 2), "duration_s": round(self.t - task.t0, 1)}
        return self._start("explore", {"max_time_s": max_time_s, **({"area": area} if area else {})}, segs, final, max_time_s)

    def patrol(self, places: list[str], loops: int = 1) -> Result:
        resolved = []
        for p in places:
            r = self._resolve(place=p)
            if isinstance(r, Result):
                return r
            resolved.append((r[2]["place"], r[0]))
        segs, p = [], (self.x, self.y)
        for _ in range(loops):
            for name, xy in resolved:
                path = self._plan_to(xy, p)
                if isinstance(path, Result):
                    return path
                segs += self._path_segments(p, path) + [("rotate", 2 * math.pi), ("mark", name)]
                p = xy

        def final(reason, task):
            visits = []
            seen_before = set(next(m for m in self.missions if m["id"] == task.id)["tracks0"])
            for mk in task.marks:
                new = [self._obj(self.tracks[i]) for i in sorted(mk["tracks"] - seen_before)]
                seen_before |= mk["tracks"]
                visits.append({"place": mk["place"], "reached": True, "new": new, "missing": [], "moved": []})
            return {"visits": visits, "loops_done": len(task.marks) // max(1, len(resolved)),
                    "travelled_m": round(task.travelled, 2), "duration_s": round(self.t - task.t0, 1)}
        return self._start("patrol", {"places": places, "loops": loops}, segs, final)

    def inspect(self, object: dict | None = None, object_id: int | None = None) -> Result:
        r = self._resolve(object=object, object_id=object_id)
        if isinstance(r, Result):
            return r
        goal, face, info = r
        path = self._plan_to(goal)
        if isinstance(path, Result):
            return path
        tr = self.tracks[info["object"]["id"]]
        old = (tr["x"], tr["y"])
        segs = self._path_segments((self.x, self.y), path) + [("rotate_to", math.atan2(face[1] - goal[1], face[0] - goal[0]))]

        def final(reason, task):
            gt = self._gt(tr["gid"])
            confirmed = gt.present and self._visible(gt) and reason in ("reached", "done")
            moved = 0.0
            if confirmed:
                self._sight(gt)
                moved = math.hypot(gt.x - old[0], gt.y - old[1])
                tr["x"], tr["y"] = gt.x, gt.y
            elif reason in ("reached", "done") and not gt.present:
                tr["state"] = "missing"
                self._emit("object_missing", "warning", f"The {tr['color']} {tr['class']} is gone.", object_id=tr["id"])
            return {"confirmed": confirmed, "object": self._obj(tr), "photo_path": None,
                    "moved_m": round(moved, 2)}
        args = {k: v for k, v in dict(object=object, object_id=object_id).items() if v is not None}
        return self._start("inspect", args, segs, final)

    def stop(self) -> Result:
        t = self.task
        if t is None:
            return ok(stopped=True, cancelled_task=None)
        tid, tool = t.id, t.tool
        self._finish(t, "cancelled")
        return ok(stopped=True, cancelled_task={"task_id": tid, "tool": tool})

    def mission_control(self, action: str) -> Result:
        t = self.task
        if t is None:
            return fail("UNAVAILABLE", "There is no active mission.")
        if action == "pause":
            t.paused = True
            self.speed = 0.0
            self._emit("task_paused", "info", f"{t.tool} paused.", task_id=t.id)
        elif action == "resume":
            t.paused = False
            self._emit("task_resumed", "info", f"{t.tool} resumed.", task_id=t.id)
        else:
            self._finish(t, "cancelled")
            return ok(state="cancelled", task={"task_id": t.id, "tool": t.tool})
        return ok(state="paused" if t.paused else "running", task={"task_id": t.id, "tool": t.tool})

    def schedule(self, delay_s: int, call: dict) -> Result:
        if call["tool"] not in TOOLS:
            return fail("UNKNOWN_TOOL", f"no tool named '{call['tool']}'")
        problems = validate(call["args"], TOOLS[call["tool"]].parameters)
        if problems:
            return fail("INVALID_ARGS", "; ".join(problems), problems=problems)
        sid = len(self.schedules) + 1 + sum(1 for e in self.events if e["type"] == "schedule_fired")
        self.schedules.append({"id": sid, "fires_at": self.t + delay_s, "call": call})
        return ok(schedule_id=sid, fires_at=round(self.t + delay_s, 1))

    # ================================================================ questions
    def where_is(self, object: dict) -> Result:
        m = self._match(object)
        if not m:
            return fail("NOT_SEEN", f"I have not seen a {object.get('color') or ''} {object['class'].replace('_', ' ')}.".replace("a  ", "a "))
        return ok(matches=[self._obj(t) for t in m])

    def whats_nearby(self, radius_m: float = 2.0) -> Result:
        objs = [self._obj(t) for t in self.tracks.values()]
        objs = sorted((o for o in objs if o["distance_m"] <= radius_m), key=lambda o: o["distance_m"])
        obstacles = []
        for k in range(8):
            b = k * math.pi / 4
            d = self.world.ray_distance((self.x, self.y), self.yaw + b, radius_m)
            if d is not None:
                obstacles.append({"bearing_word": bearing_word(math.degrees(b)), "distance_m": round(d, 2)})
        return ok(objects=objs, obstacles=obstacles, place=self.world.place_at(self.x, self.y))

    def where_am_i(self) -> Result:
        hx, hy = self.places["home"]
        return ok(x=round(self.x, 2), y=round(self.y, 2), heading_deg=round(math.degrees(self.yaw), 1),
                  place=self.world.place_at(self.x, self.y), home_distance_m=round(math.hypot(self.x - hx, self.y - hy), 2),
                  localisation="ok")

    def relation(self, a: dict, b: dict) -> Result:
        ta, err = self._unique(a)
        if err:
            return err
        tb, err = self._unique(b)
        if err:
            return err
        ang = math.degrees(_wrap(math.atan2(ta["y"] - tb["y"], ta["x"] - tb["x"]) - self.yaw))
        pa, pb = self.world.place_at(ta["x"], ta["y"]), self.world.place_at(tb["x"], tb["y"])
        return ok(a_from_b=bearing_word(ang), distance_m=round(math.hypot(ta["x"] - tb["x"], ta["y"] - tb["y"]), 2),
                  same_place=pa == pb, a=self._obj(ta), b=self._obj(tb))

    def path_check(self, place=None, object=None, object_id=None, point=None) -> Result:
        r = self._resolve(place, object, object_id, point)
        if isinstance(r, Result):
            return r
        goal, _, info = r
        in_zone = self._in_zone(*goal)
        path = None if in_zone else self.world.plan((self.x, self.y), goal, self._zone_circles())
        crosses = in_zone or (path is None and bool(self.zones) and self.world.plan((self.x, self.y), goal) is not None)
        tgt_place = self.world.place_at(*goal)
        leaves = tgt_place in self.world.rooms and tgt_place not in self.mapped_rooms
        if path is None:
            return ok(reachable=False, path_length_m=None, eta_s=None,
                      blocked_by="no-go zone" if crosses else "unknown obstacle",
                      leaves_mapped_area=leaves, crosses_no_go=crosses, target=info)
        L = self.world.path_length((self.x, self.y), path)
        return ok(reachable=True, path_length_m=round(L, 2), eta_s=round(L / V_WALK, 1), blocked_by=None,
                  leaves_mapped_area=leaves, crosses_no_go=False, target=info)

    def status(self) -> Result:
        t = self.task
        mode = "estop" if self.estop else ("paused" if t and t.paused else ("moving" if t else "idle"))
        return ok(battery_pct=round(self.battery, 1), battery_simulated=True, legs=dict(self.legs), mode=mode,
                  task={"task_id": t.id, "tool": t.tool} if t else None, speed_mps=round(self.speed, 2),
                  uptime_s=round(self.t, 1), estop=self.estop, alerts_active=list(self.alerts))

    def why(self, about: str) -> Result:
        types = {"stop": ("task_failed", "task_finished", "estop", "path_blocked"),
                 "last_task": ("task_started", "task_finished", "task_failed", "path_blocked"),
                 "alert": ("leg_failure", "battery_low", "path_blocked", "object_missing"),
                 "refusal": ("refusal",)}[about]
        evs = [e for e in self.events if e["type"] in types]
        evs = select_why(about, self.events, evs)
        return ok(events=[{k: e[k] for k in ("stamp", "type", "severity", "text")} for e in evs])

    def recall(self, what: str, query: dict | None = None) -> Result:
        query = query or {}
        if what == "object":
            out = []
            for t in self.tracks.values():
                if query.get("class") and t["class"] != query["class"]:
                    continue
                if query.get("color") and t["color"].lower() != query["color"].lower():
                    continue
                age = self.t - t["last_seen"]
                if query.get("since_s") is not None and age > query["since_s"]:
                    continue
                out.append({**self._obj(t), "age_s": round(age, 1), "stale": age > STALE_AFTER_S,
                            "state": t["state"], "map_id": self.world.map_id})
            return ok(records=out)
        if what == "mission":
            return ok(records=[{"id": m["id"], "tool": m["tool"], "state": m["state"],
                                "age_s": round(self.t - (m["end"] or m["start"]), 1)} for m in self.missions])
        return ok(records=[{"name": n, "x": xy[0], "y": xy[1], "map_id": self.world.map_id} for n, xy in self.places.items()])

    def describe_view(self, question: str | None = None) -> Result:
        vis = [self._obj(t) for t in self.tracks.values()
               if self._visible(self._gt(t["gid"])) and self._gt(t["gid"]).present]
        if vis:
            text = "I can see " + ", ".join(f"a {o['color']} {o['class'].replace('_', ' ')} ({o['bearing_word']})".replace("a  ", "a ") for o in vis) + "."
        else:
            text = "I do not see any known objects right now."
        return ok(text=text, unverified=True, detections=vis, photo_path=None)

    def report(self, mission_id: int | None = None) -> Result:
        done = [m for m in self.missions if m["end"] is not None]
        m = next((x for x in done if x["id"] == mission_id), None) if mission_id else (done[-1] if done else None)
        if m is None:
            return fail("UNAVAILABLE", "There is no finished mission to report.")
        evs = [e for e in self.events if m["start"] <= e["stamp"] <= m["end"]]
        new = [self._obj(self.tracks[i]) for i in sorted(set(self.tracks) - m["tracks0"])]
        return ok(summary={"mission_id": m["id"], "tool": m["tool"], "state": m["state"],
                           "duration_s": round(m["end"] - m["start"], 1), "distance_m": m["distance_m"],
                           "objects": new, "alerts": [e["text"] for e in evs if e["severity"] in ("warning", "critical")]},
                  report_path=None)

    # ================================================================ configuration
    def no_go_zone(self, action: str, place: str | None = None, around_object_id: int | None = None,
                   radius_m: float = 0.8) -> Result:
        if action == "list":
            return ok(zones=[dict(z) for z in self.zones])
        if action == "add":
            if place is not None:
                r = self._resolve(place=place)
                if isinstance(r, Result):
                    return r
                x, y = r[0]
                name = r[2]["place"]
            elif around_object_id is not None:
                tr = self.tracks.get(around_object_id)
                if tr is None:
                    return fail("NOT_SEEN", f"No object with id {around_object_id} is known.")
                x, y, name = tr["x"], tr["y"], f"around #{around_object_id}"
            else:
                return fail("INVALID_ARGS", "give place or around_object_id")
            zid = 1 + max((z["id"] for z in self.zones), default=0)
            self.zones.append({"id": zid, "name": name, "x": round(x, 2), "y": round(y, 2), "r": radius_m})
            return ok(zones=[dict(z) for z in self.zones])
        # remove: by place name or object id; the safety layer (not this backend) asks for confirmation
        keep = [z for z in self.zones if not ((place and _norm(z["name"]) == _norm(place))
                                              or (around_object_id and z["name"] == f"around #{around_object_id}"))]
        if len(keep) == len(self.zones):
            return fail("UNKNOWN_PLACE", "No such no-go zone.")
        self.zones = keep
        return ok(zones=[dict(z) for z in self.zones])

    def save_place(self, name: str) -> Result:
        self.places[name] = (round(self.x, 2), round(self.y, 2))
        return ok(place={"name": name, "x": round(self.x, 2), "y": round(self.y, 2)})
