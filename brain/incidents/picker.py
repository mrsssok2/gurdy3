"""LLM tool picker for incident questions (Phase 6d). The keyword router plateaued (6c: 7/20 own, 7/10 external): free phrasings fell through to the planner.

The model only CHOOSES: it returns JSON {tool, room, object, about, still} for one of the six incident tools or "none". Everything it says is validated strictly and
most of it is not trusted at all:
  * tool       must be one of INCIDENT_TOOLS or "none" (-> no plan, the existing routing continues);
  * room       must be a room of the CURRENT map (rooms are given in the prompt); a room that is not on the map rejects the pick (the existing routing says "there is no room X");
  * object     is taken from the user's own words (`named_objects`: known classes / colours), never from the model; a pick that needs an object and finds none is rejected;
  * time       (window, threshold, clock time) is parsed from the user's words by `nlu.parse_since` / `nlu.parse_threshold`, never by the model;
  * answers    are templates over the query result (the agent never lets the model word them); the model cannot print memory.
Any problem (server down, bad JSON, invalid value) returns None and the caller falls back to the keyword routing. Never raises."""
from __future__ import annotations

import json
import os
import re
import time
import urllib.request
from typing import Any, Callable

from ..agent.routing import named_objects
from . import nlu
from .tools import INCIDENT_TOOLS

ALLOWED = tuple(INCIDENT_TOOLS)
NEEDS_OBJECT = {"last_seen", "object_history"}

# ---- cheap, high-recall pre-filter: a question (or request for information) that could be about the past, changes, history or the state of a place
_MOTION = nlu._MOTION
_WORDS = re.compile(
    r"\b(?:did|does|do|has|have|had|was|were|been|is|are|am|will)\b|\b(?:chang\w*|differ\w*|new|moved?|moving|shift\w*|miss\w*|gone|vanish\w*|disappear\w*|taken|took|steal\w*|stole\w*|"
    r"remov\w*|still|anymore|before|earlier|later|ago|since|while|when|lately|recent\w*|last|latest|ever|yet|already|happen\w*|event\w*|recap|summar\w*|history|log|timeline|"
    r"visit\w*|check\w*|inspect\w*|seen|saw|spot\w*|notic\w*|stop\w*|halt\w*|paus\w*|fail\w*|reason|cause|why|overdue|neglect\w*|surpris\w*|alter\w*|story|highlights?|noteworthy|rearrang\w*|bump\w*|tamper\w*|touch\w*|astray|went|up\s+to\s+speed|odd|unusual|compar\w*|untouched|"
    r"disturb\w*|someone|somebody|anyone|anybody|anything|tell|give|show|list|describe|fill|walk\s+me|brief|update|status\s+of|what|which|where|how)\b", re.I)


_INFO_VERB = re.compile(r"^\s*(?:please\s+)?(?:run|walk|take)\s+(?:me\s+)?through\b|^\s*(?:please\s+)?(?:go|run)\s+(?:back\s+)?over\b", re.I)    # "walk me through your day" asks for information


def might_be_incident(text: str) -> bool:
    t = " ".join(text.split())
    if not t or len(t) > 240 or (_MOTION.search(t) and not _INFO_VERB.search(t)):
        return False
    return bool(_WORDS.search(t))


PROMPT = """You route questions to a robot's memory tools. Reply with ONE JSON object and nothing else:
{"tool": <name or "none">, "room": <room name or null>, "about": <"stop"|"failure"|null>, "still": <true|false>}

Tools:
- what_changed: what is different / moved / missing / new / taken, overall or in one room, compared with before.
- last_seen: when and where one named object was last seen.
- object_history: has one named object moved, where did it end up, is it still where it was.
- timeline: what happened / the events / a recap, optionally for one room.
- incident_why: why the robot stopped or a task failed (about = "stop" or "failure").
- unchecked_rooms: which rooms were not visited for a while, or when the robot was last in a given room.
Questions about something taken, stolen, missing, tampered with, shifted, new or different are what_changed (no object needed) or object_history (one named object).
last_seen and object_history need a named object; with only a room use what_changed or unchecked_rooms.
- none: anything else (commands, live questions such as what do you see, battery, where something is now, chat).

Rooms on the current map: {rooms}. Use a room only if the user names one of these exactly; otherwise null.
"still" is true only for yes/no questions about an object staying put ("is it still ...", "has it stayed ...").

Examples:
Q: has anything been altered in room B while I was at lunch
{"tool": "what_changed", "room": "room B", "about": null, "still": false}
Q: what is the latest you saw of the blue crate
{"tool": "last_seen", "room": null, "about": null, "still": false}
Q: did the extinguisher stay where it was
{"tool": "object_history", "room": null, "about": null, "still": true}
Q: give me the highlights of this session
{"tool": "timeline", "room": null, "about": null, "still": false}
Q: how come you quit moving a while back
{"tool": "incident_why", "room": null, "about": "stop", "still": false}
Q: which areas have been left alone the longest
{"tool": "unchecked_rooms", "room": null, "about": null, "still": false}
Q: when did you last set foot in room C
{"tool": "unchecked_rooms", "room": "room C", "about": null, "still": false}
Q: have any items gone astray in room A
{"tool": "what_changed", "room": "room A", "about": null, "still": false}
Q: is there anything here that wasn't here before
{"tool": "what_changed", "room": null, "about": null, "still": false}
Q: has the green drum been nudged
{"tool": "object_history", "room": null, "about": null, "still": false}
Q: is there a sofa in the hall
{"tool": "none", "room": null, "about": null, "still": false}
Q: how much battery is left
{"tool": "none", "room": null, "about": null, "still": false}
Q: where is the cone right now
{"tool": "none", "room": null, "about": null, "still": false}
"""

_ROOM_RE = re.compile(r"^\s*(?:room\s+)?([A-Za-z])\s*$")


def canon_room(value: Any, rooms: list[str]) -> str | None:
    """The map's own spelling of a room, or None (not a string, not on the map)."""
    if not isinstance(value, str) or not value.strip():
        return None
    v = value.strip().lower()
    for r in rooms:
        if r.lower() == v or r.lower() == f"room {v}":
            return r
    return None


def validate(raw: Any, text: str, rooms: list[str]) -> dict | None:
    """The plan {tool, args[, style]} for the model's answer, or None when anything is off (None = use the keyword routing)."""
    if not isinstance(raw, dict):
        return None
    tool = raw.get("tool")
    if tool not in ALLOWED:
        return None                                              # "none", an invented tool, a non-string
    # the room comes from the USER's words (the model only has to agree): a room the model adds on its own is dropped, one that contradicts the text rejects the pick
    typed = nlu._room(" ".join(text.split()))
    room = canon_room(typed, rooms) if typed else None
    if typed and room is None:
        return None                                              # a room that is not on this map: never guessed
    if raw.get("room") not in (None, "", "null"):
        said = canon_room(raw.get("room"), rooms)
        if typed and said is not None and said.lower() != room.lower():
            return None                                          # the model contradicts the user's own words
    objs = named_objects(text)
    if tool in NEEDS_OBJECT and not objs and room:
        tool = "unchecked_rooms" if tool == "last_seen" else "what_changed"      # asked about a ROOM with an object tool: the room tool answers it
    since, thr = nlu.parse_since(text), nlu.parse_threshold(text)
    args: dict[str, Any] = {}
    plan_style: dict[str, Any] = {}
    if tool == "what_changed":
        args = {"room": room, "since": since or (None if nlu._FIRST.search(text) is None else "session")}
    elif tool in NEEDS_OBJECT:
        if not objs:
            return None                                          # the object must come from the user's own words (known classes / colours)
        args = {"object": objs[0], "since": since if tool == "object_history" else None}
        if tool == "object_history":
            if raw.get("still") is True:
                plan_style["still"] = True
                if room:
                    plan_style["room"] = room
            elif re.search(r"\bwhere\b", text, re.I):
                plan_style["where"] = True
    elif tool == "timeline":
        args = {"since": since, "room": room, "limit": 10, "object": objs[0] if objs else None}
    elif tool == "incident_why":
        about = raw.get("about")
        if about not in ("stop", "failure"):
            about = "stop"
        args = {"about": about}
    elif tool == "unchecked_rooms":
        args = {"older_than_s": None if room else thr, "room": room}
    plan = {"tool": tool, "args": {k: v for k, v in args.items() if v is not None}}
    if plan_style:
        plan["style"] = plan_style
    return plan


class IncidentPicker:
    def __init__(self, model: str | None = None, host: str | None = None, timeout: float = 20.0, keep_alive: str = "30m",
                 transport: Callable[[str, str], str] | None = None):
        self.model = model or os.environ.get("GURDY_PICKER_MODEL") or "gurdy-sft-400"
        self.host = host or os.environ.get("OLLAMA_URL") or "http://127.0.0.1:11435"
        self.timeout, self.keep_alive = timeout, keep_alive
        self.transport = transport or self._ollama
        self.calls = self.failures = self.invalid = 0
        self.latencies: list[float] = []

    def _ollama(self, system: str, user: str) -> str:
        body = {"model": self.model, "stream": False, "think": False, "keep_alive": self.keep_alive, "format": "json",
                "options": {"temperature": 0, "num_ctx": 2048, "num_predict": 80},
                "messages": [{"role": "system", "content": system}, {"role": "user", "content": user}]}
        req = urllib.request.Request(self.host + "/api/chat", json.dumps(body).encode(), {"Content-Type": "application/json"})
        return json.load(urllib.request.urlopen(req, timeout=self.timeout))["message"]["content"]

    def raw(self, text: str, rooms: list[str]) -> Any:
        system = PROMPT.replace("{rooms}", ", ".join(rooms) if rooms else "none")
        t0 = time.perf_counter()
        out = self.transport(system, "Q: " + text.strip())
        self.latencies.append(time.perf_counter() - t0)
        try:
            return json.loads(out)
        except (ValueError, TypeError):
            fixed = re.search(r"\{.*\}", out or "", re.S)
            return json.loads(fixed.group(0)) if fixed else None

    def pick(self, text: str, rooms: list[str]) -> dict | None:
        """Plan for an incident question, None for anything else / any failure. Never raises."""
        if not might_be_incident(text):
            return None
        self.calls += 1
        try:
            raw = self.raw(text, rooms)
        except Exception:                                            # noqa: BLE001  server down, timeout, bad JSON: the keyword routing carries on
            self.failures += 1
            return None
        plan = validate(raw, text, rooms)
        if plan is None and isinstance(raw, dict) and raw.get("tool") in ALLOWED:
            self.invalid += 1
        return plan
