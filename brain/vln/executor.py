"""VLN execution (docs/results/vln_plan.md section 5): a validated Plan -> legs on the backend's own `go_to(point)` task
machinery, with the checks running on the robot's REAL pose. Mock backend only so far (behind the agent's `vln` flag).

  * avoided rooms -> TEMPORARY rectangular no-go zones (backend.add_temp_zone, tag per run, always removed at the end);
    an avoided START room gets its zone only once the robot has left it
  * room order: the rooms entered must follow plan.rooms; another room -> stop, one re-plan from the current pose, else
    an honest failure; an avoided room -> immediate stop (the zones should make this impossible: safety net)
  * "pass" legs: the robot must come within PASS_CHECK (1.2 m) of the landmark, else the walk stops there
  * arrival: the FINAL pose must be inside the goal region (room rectangle / point radius) AND, when the goal has a
    landmark, the landmark must be re-seen during the last leg near its map position (else one turn towards it and a
    second look); only then "I'm in room D / at the red barrel"; otherwise the reply says what is true instead
  * e-stop at any time: the backend's e-stop flag, or the running leg cancelled by someone else (AgentHost.estop calls
    the stop tool without the agent lock) -> the walk ends at once, zones removed, "I stopped ..."
Blocking: `run` returns when the walk is over. Time: MockBackend.advance (virtual clock); a real backend would sleep and
tick (not used yet). The legs' task events are consumed here (no per-leg "arrived" announcements); other alerts are
appended to the reply.
"""
from __future__ import annotations

import itertools
import math
import time
from dataclasses import dataclass, field

from .metrics import in_goal
from .plan import PASS_CHECK, validate

REOBSERVE_M = 0.5
_ids = itertools.count(1)


@dataclass
class VLNResult:
    ok: bool
    code: str
    reply: str
    trajectory: list = field(default_factory=list)      # [(t, x, y)] sampled every step (the backend's own pose)
    claims: list = field(default_factory=list)          # what the reply claims (metrics.episode)
    rooms_visited: list = field(default_factory=list)
    legs: list = field(default_factory=list)
    replans: int = 0
    alerts: list = field(default_factory=list)


class VLNExecutor:
    def __init__(self, backend, world_fn, dt: float = 0.5, leg_timeout_s: float = 400.0, max_replans: int = 1):
        self.b, self.world_fn, self.dt, self.leg_timeout_s, self.max_replans = backend, world_fn, dt, leg_timeout_s, max_replans

    # ---- backend access (mock today; the same names exist on RosBackend except pose / time)
    def _pose(self):
        b = self.b
        if hasattr(b, "_pose") and not hasattr(b, "advance"):
            return b._pose()
        return b.x, b.y, b.yaw

    def _estop(self) -> bool:
        p = getattr(self.b, "p", None)
        if p is not None and callable(getattr(p, "estop_active", None)):
            return bool(p.estop_active())
        return bool(getattr(self.b, "estop", False) is True)

    def _step(self):
        if hasattr(self.b, "advance"):
            self.b.advance(self.dt)
        else:
            time.sleep(self.dt)
            self.b.tick()
        for ev in self.b.poll_events():
            if not ev["type"].startswith("task_") and ev["type"] not in ("path_blocked", "object_new") and ev.get("severity") in ("warning", "critical"):
                self.alerts.append(ev.get("text") or ev["type"])

    def _sample(self, world):
        x, y, _ = self._pose()
        self.traj.append((round(self.b.now(), 2), round(x, 3), round(y, 3)))
        r = world.room_of(x, y)
        if r is not None and (not self.visited or self.visited[-1] != r):
            self.visited.append(r)
        return x, y, r

    # ---- main
    def run(self, plan, raw, text: str) -> VLNResult:
        self.traj, self.visited, self.alerts, self.legs = [], [], [], []
        self.last_leg_t0 = self.b.now()
        self.tag = f"vln{next(_ids)}"
        world = self.world_fn()
        self.start_room = world.room_of(*self._pose()[:2])
        self.deferred = set()
        res = None
        try:
            if plan.avoid and not callable(getattr(self.b, "add_temp_zone", None)):
                return VLNResult(False, "NO_ZONES", "I can't keep out of rooms with this backend, so I won't do that walk.")
            for r in plan.avoid:
                if r == self.start_room:
                    self.deferred.add(r)
                else:
                    self.b.add_temp_zone(world.rooms[r], self.tag, f"{r} (avoid, this walk only)")
            replans = 0
            while True:
                res = self._walk(plan, world)
                if res is not None and res.code == "WRONG_ROOM" and replans < self.max_replans:
                    replans += 1
                    world = self.world_fn()
                    p2 = validate(raw, text, world)
                    if p2.ok:
                        plan = p2
                        continue
                break
            if res is None:
                res = self._arrive(plan, world)
            res.replans = replans
            return res
        finally:
            if callable(getattr(self.b, "remove_temp_zones", None)):
                self.b.remove_temp_zones(self.tag)
            if res is not None:
                res.trajectory, res.rooms_visited, res.legs = self.traj, list(self.visited), list(self.legs)
                res.alerts = list(self.alerts)
                if self.alerts:
                    res.reply += " Also: " + " ".join(dict.fromkeys(self.alerts))

    def _stop_reply(self, why):
        x, y, _ = self._pose()
        room = self.world_fn().room_of(x, y)
        return f"I stopped ({why}). I'm in {room}." if room else f"I stopped ({why})."

    def _walk(self, plan, world):
        """All legs; None when every leg finished as planned, else the failure VLNResult."""
        self._sample(world)
        pointer = 0                                       # index in plan.rooms of the room the robot is in
        for wp in plan.waypoints:
            if self._estop():
                return VLNResult(False, "ESTOP", self._stop_reply("emergency stop"))
            r = self.b.go_to(point={"x": wp["x"], "y": wp["y"]})
            leg = {"label": wp["label"], "kind": wp["kind"], "t0": self.b.now(), "start": r.ok}
            self.legs.append(leg)
            if not r.ok:
                return VLNResult(False, "LEG_REFUSED", f"I can't get to {wp['label']}: {r.error['message'] if r.error else 'refused'}")
            tid = r.data.get("task_id")
            mine_cancel = False
            min_d = math.inf
            t0 = self.b.now()
            while self.b.task is not None:
                if self._estop():
                    self.b.stop()
                    return VLNResult(False, "ESTOP", self._stop_reply("emergency stop"))
                if self.b.now() - t0 > self.leg_timeout_s:
                    mine_cancel = True
                    self.b.stop()
                    return VLNResult(False, "TIMEOUT", f"I gave up on the way to {wp['label']}: it took too long.")
                self._step()
                x, y, room = self._sample(world)
                if self.deferred and room is not None and room not in self.deferred:
                    for d in list(self.deferred):           # left the avoided start room: from now on it is a zone too
                        self.b.add_temp_zone(world.rooms[d], self.tag, f"{d} (avoid, this walk only)")
                        self.deferred.discard(d)
                if room is not None and room != plan.rooms[pointer]:
                    if room in plan.avoid and room != self.start_room:
                        mine_cancel = True
                        self.b.stop()
                        return VLNResult(False, "NEVER_IN", f"I stopped: I was in {room}, which you asked me to avoid.")
                    if pointer + 1 < len(plan.rooms) and room == plan.rooms[pointer + 1]:
                        pointer += 1
                    else:
                        mine_cancel = True
                        self.b.stop()
                        return VLNResult(False, "WRONG_ROOM", f"I ended up in {room} instead of {plan.rooms[pointer + 1] if pointer + 1 < len(plan.rooms) else plan.rooms[pointer]}.")
                if wp.get("landmark") and wp["kind"] == "pass":
                    L = wp["landmark"]
                    min_d = min(min_d, math.hypot(x - L["x"], y - L["y"]))
            m = next((m for m in reversed(self.b.missions) if m["id"] == tid), None)
            state = m["state"] if m else "unknown"
            if state == "cancelled" and not mine_cancel:
                return VLNResult(False, "ESTOP", self._stop_reply("you stopped me"))
            if state not in ("reached", "done"):
                return VLNResult(False, "LEG_FAILED", f"I couldn't get to {wp['label']} ({state}). " + self._stop_reply("the way was blocked").replace("I stopped (the way was blocked). ", ""))
            if wp["kind"] == "pass" and wp.get("landmark") and min_d > PASS_CHECK:
                return VLNResult(False, "PASS_MISSED", f"I couldn't get within {PASS_CHECK:.1f} m of {wp['label']}, so I stopped there.")
            self.last_leg_t0 = t0
        return None

    def _landmark_now(self, lid):
        w = self.world_fn()
        return next((o for o in w.objects if o["id"] == lid), None)

    def _confirm_landmark(self, L, since):
        o = self._landmark_now(L["id"])
        seen = o is not None and (o.get("last_seen") or -1) >= since
        moved = None if o is None else math.hypot(o["x"] - L["x"], o["y"] - L["y"])
        return seen, moved

    def _arrive(self, plan, world):
        x, y, _ = self._pose()
        g = plan.goal
        if not in_goal(x, y, g, world):
            far = f"{math.hypot(x - g['x'], y - g['y']):.1f} m from the spot" if g["kind"] == "point" else f"in {world.room_of(x, y)}"
            return VLNResult(False, "NOT_AT_GOAL", f"I stopped {far}, not where you asked.")
        last = plan.waypoints[-1]
        L = last.get("landmark") or self._room_landmark(plan)
        if L is not None:
            seen, moved = self._confirm_landmark(L, self.last_leg_t0)
            if not seen:                                         # one look towards it
                yaw = self._pose()[2]
                bearing = math.degrees(math.atan2(math.sin(math.atan2(L["y"] - y, L["x"] - x) - yaw), math.cos(math.atan2(L["y"] - y, L["x"] - x) - yaw)))
                if abs(bearing) > 5 and self.b.turn("left" if bearing > 0 else "right", abs(bearing)).ok:
                    t0 = self.b.now()
                    while self.b.task is not None and self.b.now() - t0 < 60:
                        if self._estop():
                            self.b.stop()
                            return VLNResult(False, "ESTOP", self._stop_reply("emergency stop"))
                        self._step()
                        self._sample(world)
                seen, moved = self._confirm_landmark(L, self.last_leg_t0)
            name = L.get("label") or f"the {L['class'].replace('_', ' ')}"
            if not seen:
                return VLNResult(False, "LANDMARK_NOT_CONFIRMED", f"I'm in {world.room_of(*self._pose()[:2])}, but I couldn't see {name} "
                                                                  "where I remembered it, so I can't say I'm there.")
            if moved is not None and moved > REOBSERVE_M:
                return VLNResult(False, "LANDMARK_MOVED", f"{name[0].upper() + name[1:]} isn't where I remembered it: I see it "
                                                          f"{moved:.1f} m from there, so I stopped here.")
        claims = [{"type": "arrived"}]
        room = world.room_of(*self._pose()[:2])
        if g["kind"] == "room":
            reply = f"I'm in {g['room']}."
            claims.append({"type": "in_room", "room": g["room"]})
        else:
            reply = f"I'm {'at ' if last['kind'] == 'stop' and not _is_relation(last['label']) else ''}{last['label']}."
            if room:
                claims.append({"type": "in_room", "room": room})
        passed = [w for w in plan.waypoints if w["kind"] == "pass" and w.get("landmark")]
        if passed:
            reply += " On the way I went past " + " and ".join(w["label"] for w in passed) + "."
            claims += [{"type": "passed", "x": w["landmark"]["x"], "y": w["landmark"]["y"]} for w in passed]
        return VLNResult(True, "ARRIVED", reply, claims=claims)

    def _room_landmark(self, plan):
        """'The room with the red barrel': the landmark that defines the goal room is re-observed too."""
        return getattr(plan, "goal_landmark", None)


def _is_relation(label: str) -> bool:
    return label.split(" ", 1)[0] in ("left", "right", "next", "behind", "in", "between")
