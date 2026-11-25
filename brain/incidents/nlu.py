"""Deterministic recognition of incident questions ("what changed in room C today?", "when did you last see the toolbox?", ...). No LLM: clear
phrasings are routed here BEFORE the planner; everything else goes on to the planner as before. Only QUESTIONS match (a command such as
"find the toolbox" or "go to room C" never does). Returns a plan {"tool", "args"} or a Refusal-like dict {"reply": text} or None."""
from __future__ import annotations

import re
from typing import Any

from ..agent.routing import named_objects

_NUMW = {"a": 1, "an": 1, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "ten": 10, "fifteen": 15, "twenty": 20, "thirty": 30, "forty five": 45, "forty-five": 45}
_UNIT_S = {"second": 1, "minute": 60, "hour": 3600}
_ROOM = re.compile(r"\broom\s+([a-z])\b", re.I)
_OBJ_WORDS = r"(?:the\s+|a\s+|my\s+)?"

_WHY_STOP = re.compile(r"\bwhy\b.*\b(?:stop(?:ped)?|fail(?:ed)?|halt(?:ed)?|abort(?:ed)?|cancel(?:l?ed)?|gave\s+up|give\s+up|freeze|froze|fell)\b", re.I)
_EARLIER = re.compile(r"\b(earlier|before|previously|last\s+time|a\s+(?:while|bit)\s+ago|yesterday|this\s+morning|at\s+\d{1,2}(?::\d{2})?\s*(?:am|pm)?)\b", re.I)
_WHY_TASK = re.compile(r"\bwhy\b.*\b(?:the|that|your)\s+(?:last\s+)?(?:mission|patrol|task|walk|search)\b.*\b(?:fail|stop|abort|cancel)", re.I)
_UNCHECKED = re.compile(r"\b(?:which|what)\s+(?:rooms?|areas?|places?)\b.*\b(?:haven'?t|have\s+not|hasn'?t|has\s+not|didn'?t|did\s+not|not\s+been|not)\s+(?:you\s+)?(?:been\s+)?(?:checked|visited|looked|inspected|patrolled|covered|been|seen|in)\b", re.I)
_LAST_VISIT = re.compile(r"\bwhen\s+(?:did\s+you|were\s+you|was\s+the\s+last\s+time\s+you)\s+(?:last\s+)?(?:check|visit|inspect|patrol|be\s+in|been\s+in|in|were\s+in)\b|\bwhen\s+were\s+you\s+last\s+in\b", re.I)
_AWAY = re.compile(r"\bwhile\s+i\s+was\s+(?:away|gone|out|not\s+here)\b|\bwhat\s+did\s+i\s+miss\b|\bsince\s+i\s+(?:left|went\s+away)\b|\bsince\s+earlier\b", re.I)
_LAST_SEEN = re.compile(r"\bwhen\s+(?:did|have)\s+you\s+(?:last\s+|most\s+recently\s+)?(?:see|seen|spot|spotted|notice|noticed|find|found)\b|"
                        r"\bwhen\s+(?:was|were)\s+(?:the\s+)?[\w\s]{1,30}?\s+(?:last\s+)?(?:seen|spotted)\b|\blast\s+time\s+you\s+(?:saw|spotted|noticed|found)\b", re.I)
_HISTORY = re.compile(r"\b(?:has|have|had)\s+(?:the\s+|my\s+)?[\w\s]{1,30}?\s+(?:been\s+)?(?:moved|move|gone|disappeared|shifted|changed)\b|"
                      r"\bdid\s+(?:the\s+|my\s+)?[\w\s]{1,30}?\s+(?:move|go|disappear|change|shift)\b|\bis\s+(?:the\s+|my\s+)?[\w\s]{1,30}?\s+still\s+(?:in|at|where|there)\b|"
                      r"\bwhere\s+has\s+(?:the\s+|my\s+)?[\w\s]{1,30}?\s+been\b|\bhistory\s+of\s+(?:the\s+)?", re.I)
_CHANGED = re.compile(r"\bwhat(?:\s+has|\s+have|'s|\s+is)?\s+(?:changed|different|new|moved|missing|gone)\b|\banything\s+(?:changed|moved|different|new|missing|gone)\b|"
                      r"\b(?:has|have|did)\s+anything\s+(?:changed|change|moved|move|gone|disappeared|appeared)\b|\bany\s+changes\b|\bwhat\s+was\s+moved\b|\bwhat\s+went\s+missing\b", re.I)
_TIMELINE = re.compile(r"\bwhat\s+(?:happened|did\s+you\s+do|have\s+you\s+been\s+(?:doing|up\s+to)|went\s+on|has\s+been\s+(?:happening|going\s+on)|has\s+happened)\b|"
                       r"\b(?:show|give|read|tell)\s+(?:me\s+)?(?:the\s+|a\s+)?(?:timeline|log|history|recap|event\s+log)\b|\brecap\b|\btimeline\b", re.I)


def _num(tok: str) -> float | None:
    tok = tok.strip().lower()
    if tok.isdigit():
        return float(tok)
    return float(_NUMW[tok]) if tok in _NUMW else None


def parse_since(text: str) -> str | None:
    t = text.lower()
    if _AWAY.search(t):
        return "away"
    if re.search(r"\bsince\s+(?:you|i)\s+(?:started|turned|switched|booted|powered|woke)\b|\bsince\s+(?:the\s+)?(?:start|startup|boot)\b|\bthis\s+session\b|\bsince\s+you\s+started\s+me\b", t):
        return "session"
    if m := re.search(r"\bsince\s+(\d{1,2})(?::(\d{2}))?\s*(am|pm|o'?\s?clock)?\b", t):
        h, mi, ap = int(m.group(1)), int(m.group(2) or 0), (m.group(3) or "")
        if ap == "pm" and h < 12:
            h += 12
        elif ap == "am" and h == 12:
            h = 0
        if h <= 23 and mi <= 59:
            return f"clock:{h:02d}:{mi:02d}"
    if m := re.search(r"\b(?:in|over|during|within|for|from)\s+the\s+(?:last|past)\s+(?:(\d+|an?|one|two|three|four|five|six|ten|fifteen|twenty|thirty)\s+)?(second|minute|hour)s?\b", t):
        n = _num(m.group(1)) if m.group(1) else 1
        return f"ago:{int((n or 1) * _UNIT_S[m.group(2)])}"
    if m := re.search(r"\b(\d+|an?|one|two|three|five|ten|fifteen|twenty|thirty)\s+(minute|hour)s?\s+ago\b", t):
        return f"ago:{int((_num(m.group(1)) or 1) * _UNIT_S[m.group(2)] + 0)}"
    if re.search(r"\btoday\b|\bearlier\b|\bso\s+far\b|\bthis\s+morning\b", t):
        return "today"
    return None


def parse_threshold(text: str) -> float | None:
    t = text.lower()
    if m := re.search(r"\b(?:for|in|over)\s+(?:the\s+)?(?:last\s+|past\s+)?(\d+|an?|one|two|three|five|ten|fifteen|twenty|thirty|forty five)\s+(minute|hour)s?\b", t):
        return (_num(m.group(1)) or 1) * _UNIT_S[m.group(2)]
    if re.search(r"\b(?:for|in|over)\s+(?:the\s+)?(?:last\s+|past\s+)?(?:an?\s+)?hour\b", t):
        return 3600.0
    return None


def _edit1(a: str, b: str) -> bool:
    """Levenshtein distance exactly 1 (one substitution, insertion or deletion)."""
    if a == b or abs(len(a) - len(b)) > 1:
        return False
    if len(a) == len(b):
        return sum(x != y for x, y in zip(a, b)) == 1
    a, b = (a, b) if len(a) < len(b) else (b, a)
    return any(b[:i] + b[i + 1:] == a for i in range(len(b)))


_NEAR_ROOM = re.compile(r"\b([a-z]{3,5})\s+([a-z])\b", re.I)


def near_room(text: str) -> tuple[str, str] | None:
    """("roon B", "room B"): a misspelt "room" (edit distance 1) followed by a letter. The caller says what it assumed."""
    for m in _NEAR_ROOM.finditer(text):
        w = m.group(1).lower()
        if w != "room" and _edit1(w, "room") and m.group(2).upper() == m.group(2):
            return m.group(0), f"room {m.group(2).upper()}"
    return None


def _room(text: str) -> str | None:
    m = _ROOM.search(text)
    return f"room {m.group(1).upper()}" if m else None


def _check_room(room: str | None, known: list[str]) -> dict | None:
    """A reply when the named room does not exist (never answer 'nothing changed' about a room that is not on the map)."""
    rooms = [p for p in known if p.lower().startswith("room")]
    if room and rooms and room.lower() not in {r.lower() for r in rooms}:
        return {"reply": f"I don't know a room called {room}. I know: {', '.join(rooms)}."}
    return None


def parse(text: str, known_places: list[str]) -> dict | None:
    """{"tool", "args"} for an incident question, {"reply": ...} for one that cannot be answered, None for anything else."""
    t = " ".join(text.split())
    room, objs, since = _room(t), named_objects(t), parse_since(t)
    assumed = None
    if room is None and (nr := near_room(t)):
        heard, room = nr
        assumed = f"I took \"{heard}\" as {room}. "

    def plan(tool: str, **args: Any) -> dict:
        if (bad := _check_room(room, known_places)) and room and tool in ("what_changed", "timeline", "unchecked_rooms"):
            return bad
        out: dict = {"tool": tool, "args": {k: v for k, v in args.items() if v is not None}}
        if assumed and "room" in out["args"]:
            out["assumed"] = assumed
        return out

    if (_WHY_STOP.search(t) and _EARLIER.search(t)) or _WHY_TASK.search(t):
        about = "failure" if re.search(r"\b(?:fail|mission|patrol|task|search|walk)\w*\b", t, re.I) and not re.search(r"\bstop", t, re.I) else "stop"
        at = re.search(r"\bat\s+(\d{1,2})(?::(\d{2}))?\s*(am|pm)?\b", t, re.I)
        clk = None
        if at:
            h = int(at.group(1)) + (12 if (at.group(3) or "").lower() == "pm" and int(at.group(1)) < 12 else 0)
            clk = f"{h:02d}:{int(at.group(2) or 0):02d}"
        return plan("incident_why", about=about, at=clk)
    if _UNCHECKED.search(t):
        return plan("unchecked_rooms", older_than_s=parse_threshold(t))
    if _LAST_VISIT.search(t) and room:
        return plan("unchecked_rooms", room=room)
    if _AWAY.search(t):
        return plan("what_changed", room=room, since="away") if _CHANGED.search(t) else plan("timeline", since="away", room=room, limit=10)
    if _LAST_SEEN.search(t) and objs:
        return plan("last_seen", object=objs[0])
    if _HISTORY.search(t) and objs:
        return plan("object_history", object=objs[0], since=since)
    if _CHANGED.search(t):
        return plan("what_changed", room=room, since=since)
    if _TIMELINE.search(t):
        return plan("timeline", since=since, room=room, object=objs[0] if objs else None)
    return None
