"""Fetch task state machine (over a FetchMock-like backend).

    pick:     FIND -> APPROACH -> FINE_ALIGN (<= 3 tries, residual <= 2 cm) -> GRASP (<= 3 attempts) -> VERIFY_GRASP
              -> FOLD_ARM                                                 => holding
    deliver:  CARRY (speed <= 0.08 m/s) -> PLACE (release, verify empty)  => placed
                                        -> HAND_OVER (wait for the user)  => handed_over
Every transition is logged with its result; `confirmed` collects what really happened (picked, placed, handed_over,
dropped) and is the ONLY source replies may use for success claims (replies.ground).
Failure codes: NOT_FOUND, NO_PATH, NOT_REACHABLE, GRASP_FAILED, DROPPED, NOT_TAKEN, NOT_HOLDING.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .catalog import CARRY_SPEED_MPS
from .mock import HOME, room_of
from .safety import carry_speed

MAX_ALIGN, MAX_GRASP = 3, 3
# why the alignment failed when no residual was measured (VisualServo codes); words only, never a number
ALIGN_WHY = {"TARGET_LOST": "my wrist camera couldn't see it, so I couldn't measure where it is",
             "OCCLUDED": "something blocked my wrist camera's view, so I couldn't measure where it is",
             "TIMEOUT": "I ran out of time before I could measure where it is",
             None: "I couldn't measure where it is"}


class FetchAborted(Exception):
    """Raised by a backend primitive when the task must stop at once (emergency stop). FetchSession turns it into an honest
    reply; nothing after it runs."""


@dataclass
class Outcome:
    status: str                     # holding | placed | handed_over | failed
    code: str | None = None
    obj: str | None = None
    where: str | None = None        # room (or "you" / "home") where it ended up
    detail: str = ""
    log: list = field(default_factory=list)
    confirmed: set = field(default_factory=set)
    regrasps: int = 0


class FetchMachine:
    def __init__(self, robot):
        self.r = robot

    def _log(self, out: Outcome, state: str, ok: bool, detail=""):
        out.log.append({"t": round(self.r.now(), 1), "state": state, "ok": bool(ok), "detail": detail})
        return ok

    def pick(self, name: str) -> Outcome:
        out = Outcome("failed", obj=name)
        if self.r.carrying and self.r.carrying != name:
            out.code, out.detail = "BUSY", self.r.carrying
            return out
        if self.r.carrying == name:
            out.status, out.where = "holding", room_of(*self.r.robot)
            out.confirmed.add("picked")
            return out
        ok, d = self.r.find(name)
        if not self._log(out, "FIND", ok, d):
            out.code, out.detail = "NOT_FOUND", d
            return out
        ok, d = self.r.approach(name, carry_speed(0.10, False))
        if not self._log(out, "APPROACH", ok, d):
            out.code, out.detail = "NO_PATH", d
            return out
        for _ in range(MAX_ALIGN):
            ok, err = self.r.fine_align(name)          # err: measured residual (m) or None = not measured
            self._log(out, "FINE_ALIGN", ok, f"residual {err * 100:.1f} cm" if err is not None else
                      f"residual not measured ({getattr(self.r, 'align_code', None) or 'no observation'})")
            if ok:
                break
        if not ok:
            out.code = "NOT_REACHABLE"
            out.detail = (f"residual {err * 100:.0f} cm after {MAX_ALIGN} tries" if err is not None else
                          f"{ALIGN_WHY.get(getattr(self.r, 'align_code', None), ALIGN_WHY[None])}, {MAX_ALIGN} tries")
            return out
        for attempt in range(MAX_GRASP):
            if attempt > 0:
                # never re-use the estimate after a miss: the failed grasp may have pushed the object (2026-11-28 GUI take:
                # attempts 2-3 grasped 11 / 15 cm beside it). Look again; if it is no longer seen, say so.
                ok, err = self.r.fine_align(name)
                self._log(out, "RE_ALIGN", ok, f"residual {err * 100:.1f} cm" if err is not None else
                          f"not measured ({getattr(self.r, 'align_code', None) or 'no observation'})")
                if not ok:
                    out.regrasps = attempt
                    if err is None:
                        out.code, out.detail = "LOST_AFTER_MISS", f"{attempt} attempt{'s' if attempt > 1 else ''}"
                    else:
                        out.code, out.detail = "NOT_REACHABLE", f"residual {err * 100:.0f} cm after a missed grasp"
                    return out
            ok = self._log(out, "GRASP", self.r.grasp(name), f"attempt {attempt + 1}")
            if ok and self._log(out, "VERIFY_GRASP", self.r.verify_grasp(name)):
                break
            if getattr(self.r, "verify_code", None) == "UNCONFIRMED":
                # missing data is never a negative result: the robot keeps the gripper closed, no re-grasp, no release
                out.status, out.code, out.where = "holding", "GRASP_UNCONFIRMED", room_of(*self.r.robot)
                out.detail = getattr(self.r, "verify_why", "")
                return out
            ok = False
            out.regrasps += 1
        if not ok:
            out.code, out.detail = "GRASP_FAILED", f"{MAX_GRASP} attempts"
            out.regrasps = MAX_GRASP - 1
            return out
        self._log(out, "FOLD_ARM", self.r.fold_arm())
        out.status, out.where = "holding", room_of(*self.r.robot)
        out.confirmed.add("picked")
        return out

    def deliver(self, mode: str, room: str | None = None, out: Outcome | None = None) -> Outcome:
        """mode: 'user' (carry to the user + hand over), 'home' / 'room' (carry + place)."""
        out = out or Outcome("failed", obj=self.r.carrying)
        if not self.r.carrying:
            out.status, out.code = "failed", "NOT_HOLDING"
            return out
        out.obj = self.r.carrying
        dest = self.r.user if mode == "user" else HOME if mode == "home" else self.r.place_point(room)
        if mode == "user":                                  # stop 0.6 m in front of the user
            dest = (dest[0] + 0.6, dest[1])
        speed = carry_speed(0.10, True)
        assert speed <= CARRY_SPEED_MPS
        ok, d = self.r.carry_to(dest, speed)
        if not self._log(out, "CARRY", ok, str(d)):
            if d is not None:                               # dropped on the way
                out.status, out.code, out.where = "failed", "DROPPED", room_of(*d)
                out.confirmed.add("dropped")
            else:
                out.status, out.code, out.detail = "holding", "NO_PATH", room or mode
            return out
        if mode == "user":
            if self._log(out, "HAND_OVER", self.r.hand_over()):
                obj = self.r.objects[out.obj]
                obj.x, obj.y = self.r.user
                obj.with_user = True                        # no longer on the floor: not a pick-up candidate
                out.status, out.where = "handed_over", "you"
                out.confirmed.add("handed_over")
                return out
            code = getattr(self.r, "handover_code", None)   # real robot, bring_me delivery: why it was not confirmed
            if code == "DROPPED" or (code is None and not self.r.carrying):
                out.status, out.code, out.where = "failed", "DROPPED", room_of(*self.r.robot)
                out.confirmed.add("dropped")
            elif code == "DELIVERY_UNCONFIRMED":
                out.status, out.code, out.where = "failed", code, room_of(*self.r.robot)
            else:
                out.status, out.code = "holding", code or "NOT_TAKEN"
            return out
        if self._log(out, "PLACE", self.r.release()):
            out.status, out.where = "placed", "home" if mode == "home" else room
            out.confirmed.add("placed")
        return out

    def fetch(self, name: str, mode: str, room: str | None = None) -> Outcome:
        out = self.pick(name)
        if out.status != "holding" or out.code:         # e.g. GRASP_UNCONFIRMED: stop here, holding, nothing claimed
            return out
        log, conf, rg = out.log, set(out.confirmed), out.regrasps
        res = self.deliver(mode, room)
        res.log, res.confirmed, res.regrasps = log + res.log, conf | res.confirmed, rg
        return res
