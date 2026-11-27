"""The agent as an explicit LangGraph state machine (design 1.4).

route -> plan (LLM) -> safety_check -> execute -> observe -> respond (LLM) -> grounding_check -> end

* `route` handles, without the LLM: unsafe requests, the e-stop fast path, answers to a pending confirmation.
* `plan` may be called up to MAX_LLM_CALLS times per turn (only when the model sets "continue": it wants to see
  query results first). Motion is acknowledged by a template, so a plain command costs exactly one LLM call.
* `respond` only runs when there are query results to phrase; its text passes the grounding check.
"""
from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from typing import Any, TypedDict

from langgraph.graph import END, StateGraph

from ..tools import TOOLS, ToolLayer
from . import templates
from .alerts import ALERT_TYPES, AlertManager
from .capabilities import unsupported_request
from ..incidents import nlu as incident_nlu
from ..incidents.phrasing import must_include as incident_must, phrase as incident_phrase
from .executor import MissionExecutor
from .grounding import GroundedReply, collect_facts, grounded_reply, violations
from .fuzzy import suggest_object, suggest_place, suggest_verb
from .guard import _is_state_change, guard_plan, normalize_call, unknown_find_target, unknown_noun
from .llm import LLM, LLMUnavailable
from .refs import resolve_other
from .routing import route_plan, search_if_unseen
from . import rooms
from . import untrusted
from .answer_clock import AnswerClock
from .safety import ConfirmationGate, is_clear_no, is_clear_yes, SafetyChecker, Verdict
from .stopwords import is_estop_phrase, is_stop_why, is_unsafe_request     # noqa: F401  (re-exported: the e-stop text rules live in stopwords.py)

MAX_LLM_CALLS = 4
SYSTEM_PROMPT = ("You are Gurdy, a hexapod robot. Turn the user's request into tool calls, or ask / refuse. "
                 "You know no facts about the world: use tools. Never state a name, number or direction "
                 "that a tool did not return.")


class TurnState(TypedDict, total=False):
    utterance: str
    stt_confidence: float | None
    log_start: int
    llm_calls: int
    calls: list[dict]
    cont: bool
    followup_skip: set
    voice: bool
    followup: bool          # a motion plan said `continue`: plan the rest when the mission has finished
    approved: bool
    reply_parts: list[str]
    reply: str
    next: str
    has_query: bool
    grounding: dict
    verdicts: list[dict]
    events: list[dict]
    notes: list[str]


@dataclass
class TurnResult:
    reply: str
    calls: list[dict]
    verdicts: list[dict]
    llm_calls: int
    grounding: dict
    events: list[dict] = field(default_factory=list)


class Agent:
    def __init__(self, layer: ToolLayer, llm: LLM, safety: SafetyChecker | None = None,
                 executor: MissionExecutor | None = None, alerts: AlertManager | None = None, clock: AnswerClock | None = None,
                 vln=None):
        self.layer, self.llm = layer, llm
        self.vln = vln                  # brain.vln.planner.VLNPlanner or None (default OFF: navigation text goes to the planner as before)
        self.vln_last = None            # last VLN run (VLNResult) for tests / evaluation
        self.safety = safety or SafetyChecker(layer)
        self.executor = executor or MissionExecutor(layer, self.safety)
        self.clock = clock or AnswerClock()      # wall-clock timeouts for every pending question (see answer_clock.py)
        self.gate = ConfirmationGate(layer, self.safety.cfg["confirmation_timeout_s"], self.clock)
        self.replan_used = False        # at most one re-plan per user request (a re-plan's own failure is only reported)
        self.replans: list[dict] = []   # log of re-plan attempts (evaluation / debugging)
        self.alerts = alerts or AlertManager()
        self._failure_at: float | None = None    # wall time the last failure was spoken (dedupe of late path_blocked alerts)
        self.errors: list[str] = []              # internal exceptions caught by say() (tests / debugging)
        self.followup: dict | None = None        # {utterance, log_start, done}: plan the rest of the request once the mission is done
        self.pending_room: dict | None = None   # "Which room: A, B or D?" waiting for a letter: {template, expires_at}
        self.recent_room: dict | None = None    # last command that named a room: {utterance, t}; "no, Delta" corrects it
        self.last_stop: dict | None = None      # why the robot last stood still: {reason, at (wall), command?, tool?} (stop explanation)
        self.last_room: str | None = None       # letter of the room the conversation talked about last ("that room", "the other room")
        self.pending_fix: dict | None = None    # "Did you mean room B?" waiting for a yes: {options: {name: corrected text}, expires_at}
        self.graph = self._build()

    # ------------------------------------------------------------------ public API
    def _low_conf_message(self, text: str) -> str:
        """The ONE message for speech that was not understood well enough (safety.yaml `stt_low_confidence_message`)."""
        heard = text.strip().rstrip(" .,;:!?-") or text.strip()      # "Bad, odd." -> "Bad, odd" (no double full stop)
        return self.safety.cfg["stt_low_confidence_message"].format(heard=heard)

    def _is_low_conf(self, conf: float | None, text: str) -> bool:
        """Low confidence, and not a stop / yes / no / answer to a pending question (those are short and always allowed through)."""
        if conf is None or conf >= self.safety.cfg["stt_confidence_min"] or not text.strip():
            return False
        return not (self.gate.pending is not None or self.pending_fix is not None or is_estop_phrase(text)
                    or is_clear_yes(text) or is_clear_no(text))

    def _room_turn(self, text: str, conf: float | None) -> "str | TurnResult":
        """Room names by voice (see rooms.py): answer to "Which room?", "no, Delta" corrections, phonetic names, a missing letter,
        a room that does not exist. Returns the text to process, or a TurnResult."""
        timeout = self.safety.cfg["confirmation_timeout_s"]
        pr, self.pending_room = self.pending_room, None
        if pr is not None and not self.clock.expired(pr["asked_at"], timeout):
            if is_clear_no(text):
                return TurnResult("Okay, cancelled.", [], [{"outcome": "cancelled", "rule": "missing_room_letter"}], 0, {})
            ch = rooms.answer_letter(text) or rooms.fuzzy_answer(text, rooms.known_letters(self.layer.backend.known_places()))
            if ch:
                text = re.sub(r"\broom\b", f"room {ch}", pr["template"], count=1, flags=re.I)
                if conf is not None:             # heard by voice: read the choice back and wait for a yes (a wrong match cannot move the robot)
                    q = f"Room {ch}, {rooms.phonetic(ch) or ch}. Shall I go?"
                    self.pending_fix = {"options": {f"room {ch}": text}, "asked_at": self.clock.stamp()}
                    self.layer.call("ask_user", {"question": q, "options": [f"room {ch}"]})
                    return TurnResult(q, [], [{"outcome": "clarify", "rule": "room_answer_readback"}], 0, {})
        rr = self.recent_room
        if rr is not None and not self.clock.expired(rr["t"], 60.0) and not is_estop_phrase(text):
            ch = rooms.correction_letter(text)
            if ch:                                               # "no, Delta": drop what is running / asked, redo with the new room
                self.gate.clear(); self.pending_fix = None; self.followup = None
                if self.executor.active:
                    self.executor.cancel()
                    self.layer.call("stop")
                text = re.sub(r"\broom [A-Z]\b", f"room {ch}", rr["utterance"], count=1)
        text = rooms.normalize(text)
        if not is_estop_phrase(text) and self.gate.pending is None and self.pending_fix is None:
            text = rooms.bare_rooms(text)                    # room names beat the object fuzzy matcher: "go to Charlie" -> "go to room C"
        places = self.layer.backend.known_places()
        if not is_estop_phrase(text) and self.gate.pending is None:
            ref = rooms.room_reference(text, places, self.last_room)
            if ref is not None and ref.text is not None:
                text = ref.text                                  # "that room" after room B -> "room B"
            elif ref is not None:                                # no earlier room (or several candidates): ask, never guess
                self.pending_room = {"template": ref.template, "asked_at": self.clock.stamp()}
                self.layer.call("ask_user", {"question": ref.question, "options": [f"room {l}" for l in ref.letters][:6]})
                return TurnResult(ref.question, [], [{"outcome": "clarify", "rule": ref.rule}], 0, {})
        if not is_estop_phrase(text):
            q = rooms.missing_letter_question(text, places)
            if q:
                self.pending_room = {"template": text, "asked_at": self.clock.stamp()}
                self.layer.call("ask_user", {"question": q})
                return TurnResult(q, [], [{"outcome": "clarify", "rule": "missing_room_letter"}], 0, {})
            msg = rooms.unknown_room_message(text, places)
            if msg:
                return TurnResult(msg, [], [{"outcome": "none", "rule": "unknown_room"}], 0, {})
        if rooms.mentioned_letters(text):
            self.recent_room = {"utterance": text, "t": self.clock.stamp()}
            self.last_room = rooms.mentioned_letters(text)[-1]
        return text

    def answer_hint(self) -> str | None:
        """Text to bias speech recognition while a question is pending (Whisper initial_prompt): the expected answers."""
        if self.pending_room is not None and not self.clock.expired(self.pending_room["asked_at"], self.safety.cfg["confirmation_timeout_s"]):
            return rooms.answer_hint(rooms.known_letters(self.layer.backend.known_places()))
        if self.gate.pending is not None or self.pending_fix is not None:
            opts = list(self.pending_fix["options"]) if self.pending_fix else []
            return " ".join(["Yes. No. Yes, go. No, cancel. Stop."] + [o.capitalize() + "." for o in opts])
        return None

    def say(self, utterance: str, stt_confidence: float | None = None, started_at: float | None = None) -> TurnResult:
        """Never raises: an internal exception (a node crashed, a backend / database error) becomes a spoken "Something went wrong: ..." reply,
        the traceback goes to stderr (the agent log) and pending questions are dropped so the next turn starts clean."""
        try:
            steps_before = self.executor.steps
            res = self._say(utterance, stt_confidence, started_at)
            self._note_command(utterance, starts_task=self.executor.steps is not steps_before and not is_estop_phrase(utterance))
            return res
        except LLMUnavailable as e:                              # the model server is down / unreachable: a plain sentence, details in the log
            import sys
            print(f"[agent] language model unavailable: {e}", file=sys.stderr, flush=True)
            self.errors.append(f"LLMUnavailable: {e}")
            try:
                self.gate.clear()
            except Exception:                                    # noqa: BLE001
                pass
            self.pending_fix = self.pending_room = self.followup = None
            return TurnResult(e.user_text, [], [{"outcome": "error", "rule": "llm_unavailable"}], 0, {})
        except Exception as e:                                   # noqa: BLE001
            import sys
            import traceback
            traceback.print_exc(file=sys.stderr)
            self.errors.append(f"{type(e).__name__}: {e}")
            try:
                self.gate.clear()
            except Exception:                                    # noqa: BLE001
                pass
            self.pending_fix = self.pending_room = self.followup = None
            return TurnResult(f"Something went wrong: {type(e).__name__}: {e}", [], [{"outcome": "error", "rule": "internal_error"}], 0, {})

    def _note_command(self, utterance: str, starts_task: bool = False) -> None:
        """Tell the incident graph what the user said AFTER the answer (a command that started a mission anchors 'while I was away'; questions never do). Never raises."""
        note = getattr(self.layer.backend, "note_user_command", None)
        if note is not None:
            try:
                note(utterance, starts_task)
            except Exception as e:                               # noqa: BLE001
                self.errors.append(f"incident note failed: {e}")

    def _say(self, utterance: str, stt_confidence: float | None = None, started_at: float | None = None) -> TurnResult:
        """`started_at`: wall-clock (self.clock.wall) time the user STARTED speaking; pending questions are judged by that, not by
        when the transcript arrived."""
        self.clock.begin_turn(started_at)
        room = self._room_turn(utterance, stt_confidence)
        if isinstance(room, TurnResult):
            return room
        utterance = room
        fixed = self._did_you_mean(utterance, stt_confidence)
        if isinstance(fixed, TurnResult):
            return fixed
        other = self._other_turn(fixed)
        if isinstance(other, TurnResult):
            return other
        fixed = other
        if fixed != utterance and rooms.mentioned_letters(fixed):        # a confirmed room choice: "no, Delta" still works afterwards
            self.recent_room = {"utterance": fixed, "t": self.clock.stamp()}
        utterance = fixed
        st: TurnState = {"utterance": utterance, "stt_confidence": stt_confidence, "log_start": len(self.layer.log),
                         "llm_calls": 0, "calls": [], "reply_parts": [], "verdicts": [], "events": [], "grounding": {},
                         "approved": False, "has_query": False, "cont": False, "notes": [],
                         "voice": stt_confidence is not None}
        held_before = self.gate.pending
        out = self.graph.invoke(st)
        if held_before is not None and self.gate.pending is held_before and held_before.tool != "mission_step":
            self.gate.clear()                                  # a question that survived a whole turn unanswered is dead: a later "yes" confirms nothing
        if out.get("followup"):
            self.followup = {"utterance": out["utterance"], "log_start": self.executor.log_start}
        return TurnResult(templates.squash(out["reply"]), out["calls"], out["verdicts"], out["llm_calls"], out["grounding"], out["events"])

    # ---- speech-recognition repair: unknown place / object close to a known one -> ask, never run it unconfirmed
    def _did_you_mean(self, text: str, conf: float | None) -> "str | TurnResult":
        """Returns the text to process (the corrected one after a clear yes) or a TurnResult carrying the question."""
        pf, self.pending_fix = self.pending_fix, None
        if pf is not None and not self.clock.expired(pf["asked_at"], self.safety.cfg["confirmation_timeout_s"]):
            if is_clear_yes(text) and len(pf["options"]) == 1:
                return next(iter(pf["options"].values()))
            if is_clear_no(text):
                return TurnResult("Okay, cancelled.", [], [{"outcome": "cancelled", "rule": "did_you_mean"}], 0, {})
            for name, corrected in pf["options"].items():              # several options: the answer names one ("room B")
                if re.search(rf"\b{re.escape(name)}\b", text, re.I) and not is_estop_phrase(text):
                    return corrected
        if self.gate.pending is not None or is_estop_phrase(text) or is_unsafe_request(text):
            return text
        places = self.layer.backend.known_places()
        sug = suggest_verb(text, places) or suggest_place(text, places) or suggest_object(text, places)
        if sug is None:
            return text
        self.pending_fix = {"options": dict(sug.corrected), "asked_at": self.clock.stamp()}
        q = sug.question
        self.layer.call("ask_user", {"question": q, "options": sug.options[:6]})
        return TurnResult(q, [], [{"outcome": "clarify", "rule": "did_you_mean"}], 0, {})

    def _other_turn(self, text: str) -> "str | TurnResult":
        """"the other box" / "another barrel": resolve against what was just talked about (refs.py) before the planner sees it."""
        if self.gate.pending is not None or is_estop_phrase(text):
            return text

        def records(cls: str) -> list[dict]:
            try:
                return list(self.layer.backend.recall("object", {"class": cls}).data.get("records", []))
            except Exception:                                    # noqa: BLE001
                return []
        res = resolve_other(text, self.layer.focus, records)
        if res is None:
            return text
        if res.text is not None:
            return res.text
        if res.reply is not None:
            return TurnResult(res.reply, [], [{"outcome": "none", "rule": res.rule}], 0, {})
        self.pending_fix = {"options": dict(res.options), "asked_at": self.clock.stamp()}
        self.layer.call("ask_user", {"question": res.question, "options": list(res.options)[:6]})
        return TurnResult(res.question, [], [{"outcome": "clarify", "rule": res.rule}], 0, {})

    def pump(self) -> list[str]:
        """Advance the executor with backend events; returns what the robot says about them."""
        evs = self.executor.pump()
        if self.executor.state == "awaiting_confirmation" and self.executor.waiting and not self.gate.pending:
            w = self.executor.waiting
            self.gate.hold("mission_step", {"calls": self.executor.remaining_calls(), "resume": True}, w.reason)
        msgs = self._announce(evs)
        if self.followup is not None:
            if self.executor.state == "done":
                msgs += self._run_followup()
            elif self.executor.state in ("failed", "cancelled", "idle"):
                self.followup = None
        return msgs

    def _run_followup(self) -> list[str]:
        """The first part of a request was motion with `continue: true` (the rest needs its outcome): plan again now, with the
        results of the finished steps in front of the planner. Calls that repeat an already finished step are dropped
        (never walk to the same place twice)."""
        f, self.followup = self.followup, None
        done = {(st.tool, repr(st.args)) for st in self.executor.steps if st.status == "done"}
        st: TurnState = {"utterance": f["utterance"], "stt_confidence": None, "log_start": f["log_start"], "llm_calls": 0, "calls": [],
                         "reply_parts": [], "verdicts": [], "events": [], "grounding": {}, "approved": False, "has_query": False,
                         "cont": False, "notes": [], "followup_skip": done}
        try:
            out = self.graph.invoke(st)
        except Exception:                                    # noqa: BLE001  never break pump()
            return []
        return [out["reply"]] if out.get("reply_parts") and out.get("reply") else []

    FAILURE_QUIET_S = 20.0           # after a failure was spoken, a late "path blocked" alert for it stays silent

    def _announce(self, evs: list[dict]) -> list[str]:
        """One failure, one message: the same blocked goal arrives as a task_failed event, a mission_failed event and a path_blocked alert.
        The mission-level sentence is the only one spoken; the other two are dropped (also a late alert within FAILURE_QUIET_S)."""
        out: list[str] = []
        mission_failed = any(e["type"] == "mission_failed" for e in evs)
        now = self.clock.wall()
        self._note_stops(evs)
        for e in evs:
            t = e["type"]
            if t == "mission_failed":
                out += self._on_mission_failed(e)
                self._failure_at = now
            elif t == "task_failed" and (mission_failed or self.executor.active):
                continue
            elif t == "path_blocked" and (mission_failed or self.executor.active
                                          or (self._failure_at is not None and now - self._failure_at < self.FAILURE_QUIET_S)):
                continue
            elif t in ALERT_TYPES:       # alerts: rate-limited, info only when idle
                if (m := self.alerts.handle(e, idle=self.executor.state == "idle")):
                    out.append(m)
            elif (m := templates.describe_event(e)):
                out.append(m)
                if t == "task_failed":
                    self._failure_at = now
        return out

    # ---- the last reason the robot stood still ("why are you standing still?"), kept in state, never read back from a tool name
    _STOP_REASONS = {"reached": "goal_reached", "done": "goal_reached", "blocked": "blocked", "no_go_zone": "no_go", "timeout": "timeout",
                     "estop": "estop", "cancelled": "cancelled", "failed": "failed", "unreachable": "blocked"}

    def _note_stops(self, evs: list[dict]) -> None:
        now = self.clock.wall()
        for e in evs:
            t, d = e.get("type"), e.get("data", {}) or {}
            if t == "estop" and "engaged" in str(e.get("text", "")):
                self.last_stop = {"reason": "estop", "at": now}
            elif t in ("task_finished", "task_failed") and d.get("tool") not in ("rotate",):
                reason = self._STOP_REASONS.get(str(d.get("stopped_reason", "done" if t == "task_finished" else "failed")), "failed")
                ls = self.last_stop
                if reason == "cancelled" and ls and ls["reason"] in ("user_stop", "estop") and now - ls["at"] < 10.0:
                    continue                             # the backend's bookkeeping of the stop the user just asked for
                self.last_stop = {"reason": reason, "at": now, "tool": d.get("tool")}

    def _stop_explanation(self) -> str | None:
        """None = no stop on record (let the planner / `why` tool answer) or the robot is working right now."""
        ls = self.last_stop
        if ls is None or self.executor.active:
            return None
        ago = max(0, int(self.clock.wall() - ls["at"]))
        when = f"{ago} second{'s' if ago != 1 else ''} ago" if ago < 120 else f"{ago // 60} minutes ago"
        r = ls["reason"]
        if r == "user_stop":
            return f'You asked me to stop ("{ls["command"].rstrip(" .!")}"), {when}.'
        if r == "estop":
            return f"The emergency stop was engaged, {when}, so I stopped."
        if r == "goal_reached":
            return f"I stopped because I reached my goal, {when}."
        if r == "blocked":
            return f"I stopped because the way was blocked, {when}."
        if r == "no_go":
            return f"I stopped because the way leads through a no-go zone, {when}."
        if r == "timeout":
            return f"I stopped because the task ran out of time, {when}."
        if r == "cancelled":
            return f"The task was cancelled, {when}."
        return f"I stopped because the task failed, {when}."

    # ---- re-plan after a failed mission step (one LLM call, safety-checked like any plan)
    def _on_mission_failed(self, ev: dict) -> list[str]:
        failure_text = templates.describe_event(ev) or "I couldn't finish that."
        replan = getattr(self.llm, "replan", None)
        if replan is None or self.replan_used:
            return [self._with_offer(failure_text)]
        self.replan_used = True
        ctx = self.executor.failure_context()
        entries = self.layer.log[self.executor.log_start:]
        facts = collect_facts(entries, "", [self.layer.state_block(), failure_text, ctx.get("error") or ""])
        record = {"failure": ctx, "calls": None, "reply": None, "llm_calls": 0, "outcome": None}
        self.replans.append(record)
        out: dict = {}
        for attempt in (1, 2):                      # the second attempt only if the first reply was ungrounded
            fb = None if attempt == 1 else record.get("feedback")
            out = replan(SYSTEM_PROMPT, self.layer.state_block(), self.layer.schemas(), ctx, entries, fb) or {}
            record["llm_calls"] += 1
            if out.get("calls") or not out.get("reply"):
                break
            bad = violations(out["reply"], facts, self.layer.backend.known_places())
            if not bad:
                break
            record["feedback"] = "Your answer contained unsupported claims: " + ", ".join(
                f"{x['kind']} '{x['text']}'" for x in bad) + ". Use only values from the tool results."
            out = {"reply": ""}
        if out.get("calls"):
            safe = [normalize_call(c) for c in out["calls"] if not _is_state_change(normalize_call(c))]   # a re-plan never changes saved state
            if safe:
                return [failure_text] + self._start_replan(safe, record)
            record["outcome"] = "none"
            return [self._with_offer(failure_text)]
        if out.get("reply"):
            record["reply"], record["outcome"] = out["reply"], "reply"
            return [self._with_offer(out["reply"])]
        record["outcome"] = "none"
        return [self._with_offer(failure_text)]

    def _with_offer(self, text: str) -> str:
        """A failed step never silently drops the rest of the plan: say what was not done and offer it ("Shall I search from here?"); a yes runs
        exactly those steps from where the robot stands."""
        rest = [c for c in self.executor.unfinished_calls() if c["tool"] in TOOLS]
        if not rest:
            return text
        try:
            done = " or ".join(templates.not_done_phrase(c["tool"], c.get("args", {})) for c in rest)
        except (KeyError, TypeError):                           # malformed args from the planner: no offer
            return text
        q = f"{text} I haven't {done} yet. {templates.offer_question(rest[0]['tool'])}"
        self.gate.hold("resume_plan", {"calls": rest, "cont": False, "utterance": self.executor.steps[0].tool if self.executor.steps else ""}, q)
        return q

    def _start_replan(self, calls: list[dict], record: dict) -> list[str]:
        record["calls"] = calls
        for c in calls:
            v = self.safety.check(c["tool"], c.get("args", {}))
            if v.outcome == "refuse" or v.outcome == "clarify":
                record["outcome"] = f"blocked:{v.rule}"
                return [v.reason]
            if v.outcome == "confirm":
                self.gate.hold(c["tool"], {"calls": calls}, v.reason)
                record["outcome"] = f"confirm:{v.rule}"
                return [v.reason]
        record["outcome"] = "started"
        evs = self.executor.submit(calls)
        acks = [templates.ack(st.tool, st.args, st.result) for st in self.executor.steps
                if st.status in ("running", "done") and st.result is not None]
        msgs = ["I will try this instead: " + " ".join(acks)] if acks else []
        return msgs + self._announce([e for e in evs if e["type"] == "mission_failed"])

    # ------------------------------------------------------------------ graph
    def _build(self):
        g = StateGraph(TurnState)
        for name in ("route", "plan", "safety_check", "execute", "respond", "finish"):
            g.add_node(name, getattr(self, "n_" + name))
        g.set_entry_point("route")
        g.add_conditional_edges("route", lambda s: s["next"], {"plan": "plan", "execute": "execute", "finish": "finish"})
        g.add_conditional_edges("plan", lambda s: s["next"], {"safety_check": "safety_check", "respond": "respond", "finish": "finish"})
        g.add_conditional_edges("safety_check", lambda s: s["next"], {"execute": "execute", "finish": "finish"})
        g.add_conditional_edges("execute", lambda s: s["next"], {"plan": "plan", "respond": "respond", "finish": "finish"})
        g.add_edge("respond", "finish")
        g.add_edge("finish", END)
        return g.compile()

    def _entries(self, s: TurnState) -> list[dict]:
        return self.layer.log[s["log_start"]:]

    # ---- route
    def n_route(self, s: TurnState) -> dict:
        text = s["utterance"]
        if is_estop_phrase(text):
            ev = self.executor.cancel()
            self.gate.clear()
            res = self.layer.call("stop")
            self.last_stop = {"reason": "user_stop", "at": self.clock.wall(), "command": text.strip()}
            return {"reply_parts": ["Stopped." + (" " + untrusted.STOP_ONLY_NOTE if untrusted.stop_prefix(text) else "")], "next": "finish", "events": ev, "has_query": False,
                    "calls": [{"tool": "stop", "args": {}}], "verdicts": [{"outcome": "allow", "rule": "estop_fast_path"}]} \
                if res.ok else {"reply_parts": ["I could not stop."], "next": "finish"}
        # ---- Phase 7b: what the user's TEXT may do (untrusted.py). A stop always wins and is handled above; nothing below can change state or rules.
        if untrusted.stop_prefix(text):                       # "Emergency stop, then continue walking ..." = a stop, nothing else
            ev = self.executor.cancel()
            self.gate.clear()
            res = self.layer.call("stop")
            self.last_stop = {"reason": "user_stop", "at": self.clock.wall(), "command": text.strip()}
            return {"reply_parts": ["Stopped. " + untrusted.STOP_ONLY_NOTE if res.ok else "I could not stop."], "next": "finish", "events": ev, "has_query": False,
                    "calls": [{"tool": "stop", "args": {}}], "verdicts": [{"outcome": "allow", "rule": "stop_wins"}]}
        save_text = bool(_SAVE_WORDS.search(text))
        blocked = (("untrusted_text", untrusted.SPOOF_REPLY) if untrusted.spoofed_input(text) else
                   ("privacy", untrusted.PRIVACY_REPLY) if untrusted.privacy_request(text) else
                   ("sql_text", untrusted.SQL_REPLY) if (untrusted.sql_text(text) and not save_text) else
                   ("claim_request", untrusted.CLAIM_REPLY) if untrusted.claim_request(text) else
                   ("endless_motion", untrusted.ENDLESS_REPLY) if untrusted.endless_request(text) else None)
        if blocked:
            self.gate.clear()                                  # a pending question is never answered by this kind of text
            return {"reply_parts": [blocked[1]], "next": "finish", "verdicts": [{"outcome": "refuse", "rule": blocked[0]}]}
        speed_note: list[str] = []
        clean, fast = untrusted.speed_request(text)
        if fast and self.gate.pending is None:
            text, speed_note = clean, [untrusted.speed_note(untrusted.configured_max_speed())]
            s = {**s, "utterance": text}
        if self.gate.pending is None and (fr := self._fetch_route(text)) is not None:
            return fr
        if self.gate.pending is None and (inc := self._incident_route(text)) is not None:
            return inc
        if is_stop_why(text) and (why := self._stop_explanation()):
            return {"reply_parts": [why], "next": "finish", "verdicts": [{"outcome": "none", "rule": "stop_explanation"}]}
        if is_unsafe_request(text):
            return {"reply_parts": ["I will not do that."], "next": "finish",
                    "verdicts": [{"outcome": "refuse", "rule": "unsafe_request"}]}
        if self.gate.pending is None and (vr := self._vln_route(text)) is not None:
            return vr
        if self.gate.pending is None and _BARE_YES_NO.match(text):
            return {"reply_parts": ["There's nothing to confirm right now."], "next": "finish",
                    "verdicts": [{"outcome": "none", "rule": "nothing_pending"}]}
        if self.gate.pending is not None:
            held = self.gate.pending.args.get("utterance")
            same = held and _norm(held) == _norm(text) and not is_clear_no(text)      # repeating the command = yes
            status, p = self.gate.answer("yes" if same else text)
            if status in ("cancelled", "expired") and not is_clear_no(text) and not is_clear_yes(text):
                status = "new_command"                  # a different command: forget the old question, run the new one
            if status == "expired":
                return {"reply_parts": ["That confirmation has expired, so I did nothing."], "next": "finish"}
            if status == "cancelled":
                return {"reply_parts": ["Okay, cancelled."], "next": "finish"}
            if status == "new_command" and self.executor.state == "awaiting_confirmation":
                self.executor.cancel()                  # the mission that was waiting for that yes is dropped too
            if status == "confirmed":
                if p.tool == "mission_step":           # executor was waiting on its next step
                    ev = self.executor.approve_and_continue()
                    return {"reply_parts": ["Okay."], "next": "finish", "events": ev}
                return {"calls": p.args["calls"], "approved": True, "next": "execute",
                        "cont": bool(p.args.get("cont")), "utterance": p.args.get("utterance", text),
                        "verdicts": [{"outcome": "confirmed", "rule": p.reason}]}
        if self.gate.pending is None and (cap := unsupported_request(text)):
            return {"reply_parts": [cap], "next": "finish", "verdicts": [{"outcome": "none", "rule": "unsupported_capability"}]}
        noun = unknown_noun(text, self.layer.backend.known_places()) or unknown_find_target(text, self.layer.backend.known_places())
        if noun:                                   # no concept of this thing: say so, never substitute a known one (no LLM call)
            from .guard import UNKNOWN_PLACE_NOUNS
            dest = re.search(rf"\b(go|head|walk|move|navigate|drive|come|travel|get|take me|proceed|bring me)\b.*\b(to|towards|toward|into)\s+(the |a |an )?{re.escape(noun)}", text, re.I)
            if noun in UNKNOWN_PLACE_NOUNS or dest:         # a destination I do not know: ask, listing the places I do know
                places = self.layer.backend.known_places()
                q = f"I can't go there. I do not know a place or object called {noun}."
                self.layer.call("ask_user", {"question": q, "options": places[:6]})
                return {"reply_parts": [q + f" ({', '.join(places)})"], "next": "finish",
                        "verdicts": [{"outcome": "clarify", "rule": "unknown_noun"}]}
            msg = f"I can't do that. I don't know what a {noun} is, and I have not seen one."
            return {"reply_parts": [msg], "next": "finish", "verdicts": [{"outcome": "none", "rule": "unknown_noun"}]}
        return {"next": "plan", "utterance": text, "notes": speed_note}

    # ---- fetch commands (Phase 9d): routed without the planner, executed by brain.fetch's state machine, template replies
    def _fetch_route(self, text: str) -> dict | None:
        fs = getattr(self.layer.backend, "fetch_session", None)
        if fs is None:
            return None                                       # no arm / fetch backend: the planner handles the text as before
        try:
            reply = fs.handle(text)
        except Exception as e:                                # noqa: BLE001  (a fetch bug must not take the agent down)
            self.errors.append(f"fetch route failed: {e}")
            return {"reply_parts": ["Something went wrong with that fetch request."], "next": "finish",
                    "verdicts": [{"outcome": "none", "rule": "fetch_error"}]}
        if reply is None:
            return None
        return {"reply_parts": [reply], "next": "finish", "calls": list(fs.calls), "has_query": False,
                "verdicts": [{"outcome": "none", "rule": "fetch"}]}

    # ---- VLN (behind the `vln` flag): instruction -> validated plan -> walk with checks (brain/vln)
    def _vln_route(self, text: str) -> dict | None:
        if self.vln is None:
            return None
        be = self.layer.backend
        if getattr(be, "task", None) is not None:
            return None                                       # busy: the normal routing answers (busy / stop / ...)
        from ..vln.executor import VLNExecutor
        from ..vln.world import VLNWorld
        verdict = [{"outcome": "none", "rule": "vln"}]
        try:
            world = VLNWorld.from_backend(be)
            plan = self.vln.plan(text, world)
            if plan is None:
                return None                                   # not navigation / rejected: the planner handles the text as before
            if not plan.ok:
                return {"reply_parts": [plan.reply], "next": "finish", "verdicts": verdict, "has_query": False}
            res = VLNExecutor(be, lambda: VLNWorld.from_backend(be)).run(plan, self.vln.last_raw, text)
        except Exception as e:                                # noqa: BLE001  (a VLN bug must not take the agent down)
            self.errors.append(f"vln route failed: {e}")
            try:
                be.stop()
            except Exception:                                 # noqa: BLE001
                pass
            return {"reply_parts": ["Something went wrong with that walk, so I stopped."], "next": "finish",
                    "verdicts": [{"outcome": "none", "rule": "vln_error"}]}
        self.vln_last = res
        reply = res.reply if not plan.notes or not res.ok else plan.notes[0] + " " + res.reply
        return {"reply_parts": [reply], "next": "finish", "verdicts": verdict, "has_query": False,
                "calls": [{"tool": "go_to", "args": {"point": {"x": w["x"], "y": w["y"]}}} for w in plan.waypoints]}

    # ---- incident questions (temporal memory): routed without the planner, answered from the graph, grounded
    def _incident_route(self, text: str) -> dict | None:
        if not hasattr(self.layer.backend, "incident_query"):
            return None                                       # this backend keeps no timeline: the planner handles the text as before
        plan = incident_nlu.parse(text, self.layer.backend.known_places(), picker=getattr(self.llm, "incident_picker", None))
        if plan is None:
            return None
        verdict = [{"outcome": "none", "rule": "incident_query"}]
        if "reply" in plan:
            return {"reply_parts": [plan["reply"]], "next": "finish", "verdicts": verdict, "has_query": False}
        tool, args = plan["tool"], plan["args"]
        res = self.layer.call(tool, args)
        calls = [{"tool": tool, "args": args}]
        if not res.ok:
            msg = ("I did not understand that time." if res.code == "INVALID_ARGS" else
                   "I don't keep a timeline in this session." if res.code == "UNAVAILABLE" else f"I can't answer that: {res.error['message']}")
            return {"reply_parts": [msg], "next": "finish", "calls": calls, "verdicts": verdict, "has_query": False}
        entries = self.layer.log[-1:]
        template = plan.get("assumed", "") + incident_phrase(tool, res.data, {**args, **plan.get("style", {})})
        reply, g = template, {"attempts": 0, "fallback": False, "rejected": [], "violations": []}
        if getattr(self.llm, "phrases_incidents", False) and os.environ.get("GURDY_INCIDENT_LLM_PHRASE") == "1":
            # OFF by default (Phase 6b): in the first evaluation the model's rephrasing dropped facts ("room A not checked" while standing in it), looped
            # (one sentence x100) and wrote "has been seen ever since"; the template answer is built from the query result alone.
            # the LLM only PHRASES the answer: it must use values of the query result (grounding check) and mention the facts that carry the answer
            must = incident_must(tool, res.data)
            facts = collect_facts(entries, text)
            places = self.layer.backend.known_places()
            feedback, rejected = None, []
            for attempt in (1, 2):
                try:
                    cand = self.llm.respond(SYSTEM_PROMPT, text, entries, feedback)
                except LLMUnavailable:
                    raise
                except Exception:                                            # noqa: BLE001
                    break
                bad = violations(cand, facts, places) + [{"kind": "missing", "text": m} for m in must if m.lower() not in cand.lower()]
                if cand.strip() and not bad:
                    reply, g = cand.strip(), {**g, "attempts": attempt}
                    break
                rejected.append(cand)
                feedback = "Your answer had problems: " + ", ".join(f"{x['kind']} '{x['text']}'" for x in bad) + ". Use only values from the tool result."
            if reply == template:
                g = {**g, "attempts": len(rejected), "fallback": True, "rejected": rejected}
        return {"reply_parts": [reply], "next": "finish", "calls": calls, "verdicts": verdict, "has_query": False, "grounding": g}

    # ---- plan
    def n_plan(self, s: TurnState) -> dict:
        n = s["llm_calls"]
        out = self.llm.plan(SYSTEM_PROMPT, self.layer.state_block(), self.layer.schemas(), s["utterance"],
                            self._entries(s))
        n += 1
        if out.get("done") and s["has_query"]:      # the model says the results so far answer the request
            return {"llm_calls": n, "next": "respond"}
        if "calls" in out and out["calls"]:
            skip = s.get("followup_skip")
            if skip:
                out = {**out, "calls": [c for c in out["calls"] if (c["tool"], repr(c.get("args", {}))) not in skip]}
                if not out["calls"]:
                    return {"reply_parts": [], "llm_calls": n, "next": "finish"}
            routed = route_plan(s["utterance"], list(out["calls"]), self.layer.backend.known_places())
            if routed.reply is not None:             # an argument naming a place the robot does not know: say so, never guess
                return {"reply_parts": [routed.reply], "llm_calls": n, "next": "finish",
                        "verdicts": s["verdicts"] + [{"outcome": "none", "rule": routed.rule}]}
            def _seen(obj: dict) -> bool:                      # memory only, no movement, not logged as a tool call
                try:
                    return bool(self.layer.backend.where_is(obj).ok)
                except Exception:                              # noqa: BLE001
                    return True                                # cannot tell: keep the memory answer
            located, _ = search_if_unseen(s["utterance"], list(routed.calls), _seen, self.layer.backend.known_places())
            out = {**out, "calls": located}
            g = guard_plan(s["utterance"], list(out["calls"]), self.layer.focus, self._recall_classes)
            if g.reply is not None:
                outcome = "refuse" if g.rule == "unknown_tool" else "none"
                return {"reply_parts": [g.reply], "llm_calls": n, "next": "finish",
                        "verdicts": s["verdicts"] + [{"outcome": outcome, "rule": g.rule}]}
            if g.clarify:
                self.layer.call("ask_user", {"question": g.clarify})
                return {"reply_parts": [g.clarify], "llm_calls": n, "next": "finish",
                        "verdicts": s["verdicts"] + [{"outcome": "clarify", "rule": g.rule}]}
            if g.confirm:
                self.gate.hold(*g.confirm)
                return {"reply_parts": [g.confirm[2]], "llm_calls": n, "next": "finish",
                        "verdicts": s["verdicts"] + [{"outcome": "confirm", "rule": g.rule}]}
            plan_calls, notes = list(g.calls), list(g.notes)
            plan_calls, later = untrusted.drop_after_stop(plan_calls) if untrusted.says_stop(s["utterance"]) else (plan_calls, [])    # the user said stop: nothing after it runs
            # (a replacement the PLANNER adds -- "actually go to room A instead" = stop + go_to -- is not a stop request and keeps its go_to)
            if later:
                notes.append(untrusted.STOP_ONLY_NOTE)
            plan_calls, unasked = untrusted.unsupported_calls(s["utterance"], plan_calls)     # no confirmation question for an action nobody asked for
            if unasked and not plan_calls:
                return {"reply_parts": ["I did not do anything, because I did not understand what you want. Could you say it another way?"], "llm_calls": n,
                        "next": "finish", "verdicts": s["verdicts"] + [{"outcome": "none", "rule": "call_not_asked_for"}]}
            return {"calls": s["calls"] + plan_calls, "cont": bool(out.get("continue")), "llm_calls": n,
                    "notes": s.get("notes", []) + notes, "next": "safety_check"}
        if self._is_low_conf(s["stt_confidence"], s["utterance"]):      # nothing parseable and the recognition was shaky: ask again
            return {"reply_parts": [self._low_conf_message(s["utterance"])], "llm_calls": n, "next": "finish",
                    "verdicts": s["verdicts"] + [{"outcome": "clarify", "rule": "low_confidence"}]}
        text = out.get("reply", "")
        # a free-text answer from the planner (refusal / small talk) is not backed by tools: grounding applies
        gr = self._ground(lambda fb: text, s)
        return {"reply_parts": [gr.text], "llm_calls": n, "next": "finish", "grounding": _g(gr)}

    def _recall_classes(self, color: str) -> set[str]:
        try:
            recs = self.layer.backend.recall("object", {"color": color}).data.get("records", [])
            return {r["class"] for r in recs if "class" in r}
        except Exception:                                    # noqa: BLE001
            return set()

    # ---- safety
    def n_safety_check(self, s: TurnState) -> dict:
        verdicts = []
        pending_calls = [c for c in s["calls"] if not c.get("_done")]
        for c in pending_calls:
            v = self.safety.check(c["tool"], c.get("args", {}), s["stt_confidence"])
            verdicts.append({"tool": c["tool"], "outcome": v.outcome, "rule": v.rule, "reason": v.reason})
            if v.outcome == "refuse":
                return {"reply_parts": [v.reason], "verdicts": s["verdicts"] + verdicts, "next": "finish"}
            if v.outcome == "clarify":
                if v.rule == "low_confidence":
                    v.reason = self._low_conf_message(s["utterance"])
                q = v.reason + (f" ({', '.join(v.options)})" if v.options else "")
                self.layer.call("ask_user", {"question": v.reason, **({"options": v.options[:6]} if v.options else {})})
                return {"reply_parts": [q], "verdicts": s["verdicts"] + verdicts, "next": "finish"}
            if v.outcome == "confirm":
                self.gate.hold(c["tool"], {"calls": pending_calls, "cont": s["cont"], "utterance": s["utterance"]}, v.reason)
                return {"reply_parts": [v.reason], "verdicts": s["verdicts"] + verdicts, "next": "finish"}
        return {"verdicts": s["verdicts"] + verdicts, "next": "execute"}

    # ---- execute (+ observe)
    def n_execute(self, s: TurnState) -> dict:
        motion = []
        parts: list[str] = []
        has_query = False
        asked = False
        for c in s["calls"]:
            kind = TOOLS[c["tool"]].kind if c["tool"] in TOOLS else "query"
            if c["tool"] == "stop":
                ev = self.executor.cancel()
                res = self.layer.call("stop")
                parts.append("Stopped.")
                continue
            if c["tool"] == "ask_user":
                res = self.layer.call("ask_user", c.get("args", {}))
                parts.append(res.data.get("question", "") if res.ok else templates.error_text(res.to_dict()))
                asked = True
                continue
            if kind == "motion" and c["tool"] != "mission_control":
                motion.append(c)
                continue
            res = self.layer.call(c["tool"], c.get("args", {}))
            if c["tool"] == "mission_control" and res.ok:
                if c["args"]["action"] == "cancel":
                    self.executor.cancel()
                elif c["args"]["action"] == "pause":
                    self.executor.state = "paused" if self.executor.active else self.executor.state
                else:
                    self.executor.state = "running" if self.executor.state == "paused" else self.executor.state
            if kind == "query":
                has_query = True                    # phrased by the LLM (grounded)
            else:                                   # config / mission_control: template from the result
                parts.append(templates.summarize(c["tool"], c.get("args", {}), res.to_dict()))
        events: list[dict] = []
        if motion:
            if self.executor.active:
                parts += self._announce(self.executor.pump())      # a trip that has just finished must not count as "busy"
            if self.executor.active:                               # a new motion command replaces the running one (stop is handled earlier)
                events += self.executor.cancel()
                self.layer.call("stop")
                self.followup = None
                parts.append("Okay, switching to: " + re.sub(r"[\s.!?,;:]+$", "", s["utterance"].strip()).lower() + ".")
            if motion:
                self.replan_used = False
                events = self.executor.submit(motion, approved=s["approved"])
                for step in self.executor.steps:
                    if step.status in ("running", "done") and step.result is not None and step.tool not in ("stop",):
                        # a find that is answered from memory finishes inside submit(): say the RESULT (where, how far), not "Looking for"
                        done = next((e for e in events if e["type"] in ("task_finished", "task_failed") and step.tool == "find"
                                     and e["data"].get("task_id") == getattr(step, "task_id", None) and e["data"].get("tool") == "find"), None)
                        ack = templates.describe_event(done) if done else templates.ack(step.tool, step.args, step.result)
                        pl = step.args.get("place", "")
                        if s.get("voice") and step.tool == "go_to" and not done and re.fullmatch(r"room [A-Z]", pl) and rooms.phonetic(pl[-1]):
                            ack = ack.replace(f"Going to {pl}", f"Going to {pl}, {rooms.phonetic(pl[-1])}", 1)   # read the letter back
                        parts.append(ack)
                    elif step.status == "failed":
                        parts.append(templates.error_text(step.result) if step.result and step.result.get("error") else "I could not start that.")
                later = []
                for st in self.executor.steps:
                    if st.status == "pending":
                        try:
                            later.append(templates.preview(st.tool, st.args))
                        except (KeyError, TypeError, ValueError):       # malformed args from the planner: the step fails on its own later
                            pass
                if later and self.executor.state == "running":        # say the rest of the plan, so it is not lost to the listener
                    parts.append("Next: " + "; then ".join(later) + ".")
                if self.executor.state == "awaiting_confirmation" and self.executor.waiting:
                    self.gate.hold("mission_step", {"calls": self.executor.remaining_calls(), "resume": True},
                                   self.executor.waiting.reason)
                    parts.append(self.executor.waiting.reason)
        upd = {"followup": bool(s["cont"] and motion and not has_query and self.executor.active),
               "reply_parts": s["reply_parts"] + parts, "events": s["events"] + events, "has_query": has_query,
               "calls": [{**c, "_done": True} for c in s["calls"]]}
        if s["cont"] and s["llm_calls"] < MAX_LLM_CALLS and has_query and not asked:
            return {**upd, "cont": False, "next": "plan"}
        return {**upd, "next": "respond" if has_query and not asked else "finish"}

    # ---- respond + grounding
    def n_respond(self, s: TurnState) -> dict:
        entries = [e for e in self._entries(s) if e["tool"] not in ("stop",)
                   and not (TOOLS[e["tool"]].kind == "motion" and e["tool"] != "mission_control")]

        def gen(fb):
            return self.llm.respond(SYSTEM_PROMPT, s["utterance"], entries, fb)
        gr = self._ground(gen, s, entries)
        return {"reply_parts": [p for p in s["reply_parts"] if p] + [gr.text], "grounding": _g(gr),
                "llm_calls": s["llm_calls"] + gr.attempts}

    def _ground(self, gen, s: TurnState, entries: list[dict] | None = None) -> GroundedReply:
        entries = self._entries(s) if entries is None else entries
        return grounded_reply(gen, entries, s["utterance"], self.layer.backend.known_places(),
                              extra_facts=[self.layer.state_block()])

    def n_finish(self, s: TurnState) -> dict:
        reply = " ".join([p for p in s.get("reply_parts", []) if p] + s.get("notes", [])) or "Okay."
        did = any(e["result"]["ok"] and e["tool"] != "ask_user" for e in self._entries(s)) or bool(s.get("events")) or self.gate.pending is not None
        reply = untrusted.vacuous_ack(reply, s["utterance"], did) or reply     # a bare "Okay." when nothing was done is a false acknowledgement
        return {"reply": reply, "calls": [{k: v for k, v in c.items() if k != "_done"} for c in s.get("calls", [])]}


_SAVE_WORDS = re.compile(r"\b(save|remember|store|mark|name|call|label)\b", re.I)
_BARE_YES_NO = re.compile(r"^\W*(yes|yeah|yep|yup|sure|no|nope|nah)\W*$", re.I)    # a lone answer with no question pending


def _norm(t: str) -> str:
    return re.sub(r"[^a-z0-9 ]", "", t.lower()).strip()


def _g(gr: GroundedReply) -> dict:
    return {"attempts": gr.attempts, "fallback": gr.fallback, "rejected": gr.rejected, "violations": gr.violations}
