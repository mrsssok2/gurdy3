"""Deterministic safety rules on the *resolved* tool call (design 4.3, level 3). No LLM, no ROS.

`SafetyChecker.check(tool, args)` returns a `Verdict`: allow / confirm / refuse / clarify, with a reason that is
safe to say to the user. A `confirm` verdict is turned into a `PendingConfirmation` that holds the exact call:
"yes" executes that call and nothing else, anything other than a clear yes cancels it, it expires after 30 s.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from ..tools import TOOLS, ToolLayer
from ..tools.schema import validate
from ..tools.specs import public_args
from ..tools.types import Result

DEFAULT_CONFIG = Path(__file__).with_name("safety.yaml")

from .stopwords import is_unsafe_request   # noqa: E402,F401  (the pattern lives in stopwords.py: no heavy imports there)


def _batt(st: dict) -> float:
    """Battery percent; unknown (None, no battery node yet) counts as full: no rule fires on a number we do not have."""
    b = st.get("battery_pct")
    return 100.0 if b is None else b



@dataclass
class Verdict:
    outcome: str                      # allow | confirm | refuse | clarify
    reason: str = ""
    code: str | None = None           # error code of the tool that caused it (AMBIGUOUS, UNKNOWN_PLACE, ...)
    options: list[str] = field(default_factory=list)   # clarify candidates
    rule: str = ""                    # machine-readable rule id (for tests, logs, evaluation)

    @property
    def allowed(self) -> bool:
        return self.outcome == "allow"


def _allow() -> Verdict:
    return Verdict("allow")


class SafetyChecker:
    def __init__(self, layer: ToolLayer, config: dict | None = None):
        self.layer = layer
        self.backend = layer.backend
        self.cfg = config if config is not None else yaml.safe_load(DEFAULT_CONFIG.read_text())

    # ------------------------------------------------------------------ main entry
    def check(self, tool: str, args: dict[str, Any] | None = None, stt_confidence: float | None = None) -> Verdict:
        args = args or {}
        if tool not in TOOLS:
            return Verdict("refuse", "That is not something I can do.", rule="outside_tool_list")
        if validate(public_args(tool, args), TOOLS[tool].parameters):
            # malformed arguments (LLM output is untrusted): nothing to judge here. The tool layer rejects the call
            # (INVALID_ARGS) before any backend code runs, so nothing unsafe can execute.
            return Verdict("allow", rule="invalid_args_rejected_by_tool_layer")
        if (stt_confidence is not None and stt_confidence < self.cfg["stt_confidence_min"] and tool not in self.cfg["always_allowed"]
                and TOOLS[tool].kind != "query"):
            return self._low_confidence(tool, args)
        if tool in ("stop", "mission_control", "ask_user"):
            return _allow()
        if tool == "no_go_zone":
            return self._check_zone(args)
        if tool == "schedule":
            inner = args["call"]
            return self.check(inner["tool"], inner.get("args", {}))
        if tool not in self.cfg["motion_tools"]:
            return _allow()                                   # questions, save_place
        v = self._check_vehicle_state(tool)
        if v:
            return v
        if tool == "walk":
            return _allow()                                   # short, relative; the gate node does the pose check
        if tool == "turn":
            return _allow()
        if tool == "return_home":
            return self._check_target(tool, {"place": "home"}, allow_hazard_zone_check=True)
        if tool == "patrol":
            return self._check_patrol(args)
        if tool in ("explore", "find"):
            return self._check_search(tool, args)
        return self._check_target(tool, args)

    def _low_confidence(self, tool: str, args: dict) -> Verdict:
        """Uncertain speech recognition. A clean parse (every place known; schema already validated) is read back and needs a yes;
        otherwise the one "I heard ... say that again" message (`{heard}` is filled in by the agent)."""
        from .templates import question_for, readback
        place = args.get("place")
        clean = tool != "schedule" and (place is None or place in self.backend.known_places() or place == "home")
        if not clean:
            return Verdict("clarify", self.cfg["stt_low_confidence_message"], rule="low_confidence")
        return Verdict("confirm", f"{readback(tool, args)}. {question_for(tool)}", rule="low_confidence")

    # ------------------------------------------------------------------ rules
    def _check_vehicle_state(self, tool: str) -> Verdict | None:
        st = self.backend.status().data
        if st.get("estop"):
            return Verdict("refuse", "The emergency stop is active; release it first.", rule="estop_active")
        failed = [n for n, s in st.get("legs", {}).items() if s == "failed"]
        critical = _batt(st) < self.cfg["critical_battery_pct"]
        if (failed or critical) and tool != "return_home":
            why = "a leg has failed" if failed else "the battery is critical"
            return Verdict("refuse", f"I will only go home because {why}.", rule="degraded_motion")
        return None

    def _low_battery(self) -> bool:
        return _batt(self.backend.status().data) < self.cfg["confirm_battery_pct"]

    def _check_target(self, tool: str, args: dict, allow_hazard_zone_check: bool = False) -> Verdict:
        target = {k: args[k] for k in ("place", "object", "object_id", "point") if k in args}
        r = self.layer.backend.path_check(**target)
        if not r.ok:
            return self._from_error(r, target)
        d = r.data
        if d.get("crosses_no_go"):
            return Verdict("refuse", "That is inside or behind a no-go zone.", code="IN_NO_GO_ZONE", rule="no_go_zone")
        if not d.get("reachable"):
            return Verdict("refuse", "I cannot find a path there; it is blocked or in unknown space.", code="NO_PATH",
                           rule="unreachable")
        near = self._person_near(args, d)
        if near:
            return near
        if d.get("path_length_m", 0) > self.cfg["confirm_path_longer_m"]:
            return Verdict("confirm", f"That is {d['path_length_m']:.1f} metres away. Shall I go?", rule="long_path")
        if d.get("leaves_mapped_area"):
            return Verdict("confirm", "That is outside the area I have mapped. Shall I go?", rule="unmapped_goal")
        return _allow()

    def _person_near(self, args: dict, path_data: dict) -> Verdict | None:
        """Refuse targets closer than person_min_distance_m to a mannequin."""
        tgt = path_data.get("target", {})
        goal = None
        if tgt.get("kind") == "point":
            goal = (tgt["x"], tgt["y"])
        elif tgt.get("kind") == "place":
            goal = (tgt["x"], tgt["y"])
        elif tgt.get("kind") == "object":
            o = tgt["object"]
            if o["class"] == "mannequin" and args.get("standoff_m", 0.6) < self.cfg["person_min_distance_m"]:
                return Verdict("refuse", "I keep at least one metre away from people.", rule="person_distance")
            return None
        if goal is None:
            return None
        recs = self.backend.recall("object", {"class": "mannequin"}).data.get("records", [])
        for m in recs:
            if math.hypot(goal[0] - m["x"], goal[1] - m["y"]) < self.cfg["person_min_distance_m"]:
                return Verdict("refuse", "That spot is too close to a person.", rule="person_distance")
        return None

    def _check_search(self, tool: str, args: dict) -> Verdict:
        if tool == "explore" and args.get("area"):
            if args["area"] not in self.backend.known_places():
                return Verdict("clarify", f"I do not know a place called {args['area']}.", code="UNKNOWN_PLACE",
                               options=self.backend.known_places(), rule="unknown_place")
        if tool == "explore" and self._low_battery():
            return Verdict("confirm", "The battery is below 30 percent. Explore anyway?", rule="low_battery_search")
        return _allow()

    def _check_patrol(self, args: dict) -> Verdict:
        for p in args["places"]:
            if p == "all":
                continue
            r = self.backend.path_check(place=p)
            if not r.ok:
                return self._from_error(r, {"place": p})
            if r.data.get("crosses_no_go"):
                return Verdict("refuse", f"{p} is inside a no-go zone.", code="IN_NO_GO_ZONE", rule="no_go_zone")
        if self._low_battery():
            return Verdict("confirm", "The battery is below 30 percent. Patrol anyway?", rule="low_battery_search")
        if args.get("loops", 1) > self.cfg["confirm_patrol_loops_over"]:
            return Verdict("confirm", f"That is {args['loops']} loops. Shall I start?", rule="many_loops")
        return _allow()

    def _check_zone(self, args: dict) -> Verdict:
        if args["action"] == "remove":
            return Verdict("confirm", "Remove that no-go zone? It was set for safety.", rule="remove_no_go")
        return _allow()

    def _from_error(self, r: Result, target: dict) -> Verdict:
        code, msg = r.code, r.error["message"]
        if code == "AMBIGUOUS":
            opts = [f"{c.get('color', '')} {c['class']} in {c['place']}".strip() for c in r.error.get("candidates", [])]
            return Verdict("clarify", f"{msg} Which one?", code=code, options=opts, rule="ambiguous")
        if code == "UNKNOWN_PLACE":
            return Verdict("clarify", msg, code=code, options=r.error.get("known_places", []), rule="unknown_place")
        if code == "NOT_SEEN":
            return Verdict("clarify", f"{msg} Should I look for it?", code=code, rule="not_seen")
        if code == "IN_NO_GO_ZONE":
            return Verdict("refuse", msg, code=code, rule="no_go_zone")
        return Verdict("refuse", msg, code=code, rule="target_error")


# ====================================================================== confirmation
_YES = re.compile(r"^\s*(?:(?:yes|yeah|yep|yup|sure|ok|okay|correct|confirm(?:ed)?|affirmative|proceed|"
                  r"go ahead|do it|please|thanks|thank you)\b[\s,.!]*)+$", re.I)


_NO = re.compile(r"^\s*(?:(?:no|nope|nah|negative|cancel|never ?mind|don'?t|do not|wrong|not that)\b[\s,.!]*)+$", re.I)


def is_clear_no(text: str) -> bool:
    """An unambiguous no ("no", "nope", "never mind"). A plain command is not a no."""
    return bool(_NO.match(text))


def is_clear_yes(text: str) -> bool:
    """Only an unambiguous yes counts ("yes but go left" does not)."""
    return bool(_YES.match(text))


@dataclass
class PendingConfirmation:
    tool: str
    args: dict
    reason: str
    asked_at: float                  # wall clock (AnswerClock) when the question was created

    def as_dict(self) -> dict:
        return {"tool": self.tool, "args": self.args, "reason": self.reason}


class ConfirmationGate:
    """Holds at most one pending call; mirrors it into `layer.pending_confirmation` for the prompt state block."""

    def __init__(self, layer: ToolLayer, timeout_s: float = 30.0, clock=None):
        from .answer_clock import AnswerClock
        self.layer, self.timeout_s, self.clock = layer, timeout_s, clock or AnswerClock()
        self.pending: PendingConfirmation | None = None

    def hold(self, tool: str, args: dict, reason: str) -> PendingConfirmation:
        self.pending = PendingConfirmation(tool, dict(args), reason, self.clock.stamp())
        self.layer.pending_confirmation = self.pending.as_dict()
        return self.pending

    def clear(self) -> None:
        self.pending = None
        self.layer.pending_confirmation = None

    def answer(self, text: str) -> tuple[str, PendingConfirmation | None]:
        """-> ("none" | "expired" | "confirmed" | "cancelled", the held call). Always clears the pending call."""
        p = self.pending
        if p is None:
            return "none", None
        self.clear()
        if self.clock.expired(p.asked_at, self.timeout_s):
            return "expired", p
        return ("confirmed" if is_clear_yes(text) else "cancelled"), p
