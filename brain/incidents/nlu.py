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

_WHY_STOP = re.compile(r"\b(?:why|how\s+come|what\s+(?:made|caused|stopped|was\s+the\s+(?:reason|cause))|the\s+(?:reason|cause))\b.*"
                       r"\b(?:stop(?:ped|ping)?|fail(?:ed|ing)?|halt(?:ed|ing)?|abort(?:ed|ing)?|cancel(?:l?ed|ing)?|paus(?:e|ed|ing)|quit(?:ting)?|gave\s+up|give\s+up|freez\w*|froze|fell|ceas\w*)\b", re.I)
_EARLIER = re.compile(r"\b(earlier|before|previously|after\s+i\s+(?:said|gave|told|typed|asked)|when\s+i\s+(?:said|told|asked)|that\s+(?:order|command|request)|last\s+time|a\s+(?:while|bit)\s+ago|yesterday|this\s+morning|at\s+\d{1,2}(?::\d{2})?\s*(?:am|pm)?)\b", re.I)
_WHY_TASK = re.compile(r"\bwhy\b.*\b(?:the|that|your)\s+(?:last\s+)?(?:mission|patrol|task|walk|search)\b.*\b(?:fail|stop|abort|cancel)", re.I)
_UNCHECKED = re.compile(r"\b(?:which|what)\s+(?:rooms?|areas?|places?)\b.*\b(?:haven'?t|have\s+not|hasn'?t|has\s+not|didn'?t|did\s+not|not\s+been|not)\s+(?:you\s+)?(?:been\s+)?(?:checked|visited|looked|inspected|patrolled|covered|been|seen|in)\b", re.I)
_LAST_VISIT = re.compile(r"\bwhen\s+(?:did\s+you|were\s+you|was\s+the\s+last\s+time\s+you)\s+(?:last\s+)?(?:check|visit|inspect|patrol|be\s+in|been\s+in|in|were\s+in)\b|\bwhen\s+were\s+you\s+last\s+in\b", re.I)
_AWAY = re.compile(r"\bwhile\s+i\s+was\s+(?:away|gone|out|not\s+here)\b|\bwhat\s+did\s+i\s+miss\b|\bsince\s+i\s+(?:left|went\s+away)\b|\bsince\s+earlier\b|\b(?:the\s+way|as|how|like)\s+i\s+left\b|\bbefore\s+i\s+(?:left|went)\b|\bsince\s+i\s+(?:stepped|went|walked)\s+(?:out|away)\b", re.I)
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


# Vague relative time. DOCUMENTED DEFAULT: "a while / recently / lately / for a bit / of late" = the last VAGUE_S seconds (10 minutes). No time word at all = the tool's
# own default (what changed: today; not checked: 1 hour). Used by parse_since AND parse_threshold, so every routing path (exact patterns and the classifier) agrees.
VAGUE_S = 600.0
_VAGUE = re.compile(r"\b(?:in|for|after)\s+a\s+(?:while|bit|little\s+while|short\s+while|few\s+minutes)\b|\b(?:recently|lately|of\s+late|these\s+days|for\s+ages|in\s+ages|"
                    r"for\s+some\s+time|a\s+(?:bit|while)\s+ago|just\s+now|a\s+moment\s+ago)\b", re.I)


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
    if _VAGUE.search(t):
        return f"ago:{int(VAGUE_S)}"
    return None


_NUMTOK = r"(\d+|an?|one|two|three|four|five|six|seven|eight|nine|ten|fifteen|twenty|thirty|forty five|forty-five|sixty)"
_NUMW.update({"four": 4, "seven": 7, "eight": 8, "nine": 9, "sixty": 60})


def parse_threshold(text: str) -> float | None:
    """'for / in / over / past / more than / longer than / at least N minutes|hours' -> seconds; vague words ('a while', 'lately') -> VAGUE_S; else None."""
    t = text.lower()
    if m := re.search(r"\b(?:for|in|over|within|past|last|than|least|beyond|after)\s+(?:the\s+)?(?:last\s+|past\s+)?(?:more\s+than\s+|longer\s+than\s+|at\s+least\s+)?" + _NUMTOK + r"\s+(minute|hour|second)s?\b", t):
        return (_num(m.group(1)) or 1) * _UNIT_S[m.group(2)]
    if re.search(r"\b(?:for|in|over|than|least)\s+(?:the\s+)?(?:last\s+|past\s+)?(?:an?\s+)?hour\b", t):
        return 3600.0
    if _VAGUE.search(t):
        return VAGUE_S
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


# ---- second stage: cue-word classifier for phrasings the exact patterns above do not cover. Picks the TOOL and the ARGUMENTS only (object, room, window,
# threshold), all validated by the tool layer's schema; the answer is always a template over the query result, never raw memory.
_MOTION = re.compile(r"^\s*(?:please\s+)?(?:go|walk|drive|head|navigate|come|turn|rotate|patrol|explore|find|search|look\s+for|bring|fetch|follow|stop|halt|wait|stand|sit|"
                     r"return|move\s+(?:to|forward|back|left|right)|set|add|remove|clear|save|take\s+me|run|start|begin|resume|cancel)\b", re.I)
_C_CHANGE = re.compile(r"\b(?:miss(?:ed)?|relocat\w*|reposition\w*|transferr?\w*|touch\w*|chang\w*|differ\w*|new|surpris\w*|disturb\w*|tamper\w*|untouched|undisturbed|unchanged|intact|amiss|news|moved?|moving|shift\w*|displac\w*|missing|gone|vanish\w*|disappear\w*|taken|removed?|rearrang\w*|swapp\w*|"
                       r"stay(?:ed)?\s+put|unusual|odd|strange|out\s+of\s+place|compar\w*|diff|went\s+missing|go\s+missing|got\s+moved)\b", re.I)
_C_LAST = re.compile(r"\blast\s+(?:time|saw|seen|spotted|noticed|sighting)|\b(?:laid|set|put)\s+eyes\b|\beyes\s+on\b|\bmost\s+recent(?:ly)?\b|\blatest\s+sighting|"
                     r"\b(?:seen|spotted|noticed|saw)\b.*\b(?:recently|lately|today|earlier|ago)\b|\bhow\s+long\s+ago\b.*\b(?:see|saw|seen|spot)", re.I)
_C_HIST = re.compile(r"\bend(?:ed)?\s+up\b|\bstay(?:ed)?\s+(?:put|in\s+place|there|where)\b|\bsame\s+(?:place|spot)\b|\bstill\s+(?:in|at|where|there|here)\b|\bwhere\s+(?:it|they|he|she)\s+was\b|"
                     r"\banymore\b|\bwhere\s+(?:i|you)\s+(?:left|put|saw)\s+(?:it|them)\b|\bin\s+(?:its|their)\s+(?:place|spot)\b|\bwhere\s+.*\b(?:before|originally|first|used\s+to)\b|\bmoved?\b|\bshift\w*\b", re.I)
_C_TIME = re.compile(r"\bhappen\w*|\bevents?\b|\bsummary\b|\bsummari[sz]e\b|\brecap\b|\blog\b|\btimeline\b|\bwent\s+on\b|\bbeen\s+up\s+to\b|\bhistory\b|\bdid\s+you\s+do\b|\bactivity\b", re.I)
_C_UNCH = re.compile(r"\bneglect\w*|\bunvisited\b|\bunchecked\b|\buninspected\b|(?:\bhaven'?t|\bhave\s+not|\bhasn'?t|\bnot\b|\bnever\b|\blongest|\bneglect\w*|\boverdue|\bstale|\bignor\w*).*\b(?:check\w*|visit\w*|look\w*|inspect\w*|patrol\w*|cover\w*|been\s+(?:in|to)|seen|surveyed)\b|"
                     r"\bwhich\s+(?:room|area|place)s?\b.*\b(?:last|longest|oldest|while|ages)\b", re.I)
_EXISTS = re.compile(r"\b(?:is|are)\s+there\s+(?:a|an|any)?\s*(?:room|area)\s+([a-z])\b|\bdo\s+you\s+(?:have|know)\s+(?:a\s+)?room\s+([a-z])\b|\bdoes\s+room\s+([a-z])\s+exist\b", re.I)
_STATE = re.compile(r"\b(?:the\s+way|as|how)\s+i\s+left\b|\bsame\s+as\b|\bas\s+(?:it|they)\s+was\b|\blike\s+(?:before|it\s+was)\b|\bversus\b|\bvs\b|\bcompared?\b|"
                    r"\banything\s+(?:off|wrong|odd|suspicious|amiss)\b|\bhow\s+(?:it|things)\s+(?:was|were|looked)\b", re.I)
_VISIT = re.compile(r"\b(?:when|what\s+time|how\s+long)\b.*\b(?:you|were|was)\b.*\b(?:in|at|through|into|visit\w*|enter\w*|pass\w*|check\w*|inspect\w*|been|came|come|stop\w*|drop\w*|went|there)\b|"
                    r"\blast\s+(?:time|visit)\b.*\b(?:in|at|through|into|visit\w*|enter\w*|pass\w*|check\w*|been)\b|\bwhen\s+were\s+you\b", re.I)
_FIRST = re.compile(r"\bbefore\b.*\b(?:now|versus|vs|today|currently)\b|\bversus\b|\bvs\b|\bthen\s+and\s+now\b|\b(?:first\s+time|(?:at|with|to|from)\s+the\s+(?:start|beginning)|when\s+you\s+(?:started|began|first)|originally|to\s+begin\s+with|at\s+first)\b", re.I)
_MOVELIST = re.compile(r"\b(?:which|what|list|name|show)\b.*\b(?:things?|objects?|items?|stuff|ones)\b.*\b(?:moved|changed|shifted|missing|gone|taken|different)\b|\b(?:list|name)\s+(?:what|everything)\b", re.I)


def _rooms_text(known: list[str]) -> str:
    letters = sorted(p.split()[-1] for p in known if re.fullmatch(r"room [A-Z]", p))
    return (letters[0] if len(letters) == 1 else ", ".join(letters[:-1]) + " and " + letters[-1]) if letters else ""


def classify(text: str, known_places: list[str]) -> dict | None:
    """Plan / {"reply"} for an incident-type question the exact patterns missed, None when the text is not about the timeline at all."""
    t = " ".join(text.split())
    if _MOTION.search(t):
        return None
    room, objs, since = _room(t), named_objects(t), parse_since(t)
    assumed = None
    if room is None and (nr := near_room(t)):
        heard, room = nr
        assumed = f"I took \"{heard}\" as {room}. "

    def plan(tool: str, **args: Any) -> dict:
        if room and (bad := _check_room(room, known_places)) and tool in ("what_changed", "timeline", "unchecked_rooms"):
            return bad
        out: dict = {"tool": tool, "args": {k: v for k, v in args.items() if v is not None}}
        if assumed and "room" in out["args"]:
            out["assumed"] = assumed
        return out

    if m := _EXISTS.search(t):
        ch = next(g for g in m.groups() if g).upper()
        rooms = [p for p in known_places if p.lower().startswith("room")]
        if f"room {ch}" in rooms:
            return {"reply": f"Yes, room {ch} is on my map. My rooms are {_rooms_text(known_places)}."}
        return {"reply": f"No, there is no room {ch}. My rooms are {_rooms_text(known_places)}."}
    if _C_UNCH.search(t) and not objs:
        thr = parse_threshold(t)
        return plan("unchecked_rooms", older_than_s=thr, room=room if re.search(r"\bwhen\b", t, re.I) else None)
    first = since if since else ("session" if _FIRST.search(t) else None)
    if room and not objs and _VISIT.search(t) and not _C_CHANGE.search(t):
        return plan("unchecked_rooms", room=room)                  # "when / what time were you last in room B": the last visit
    if objs:
        o = objs[0]
        if _C_LAST.search(t) or (re.search(r"\b(?:when|how\s+long)\b", t, re.I) and re.search(r"\b(?:see|saw|seen|spot\w*|notic\w*)\b", t, re.I)):
            return plan("last_seen", object=o)
        if _C_HIST.search(t) or _C_CHANGE.search(t) or _STATE.search(t):
            return plan("object_history", object=o, since=first)
        return None
    if _C_CHANGE.search(t) or _STATE.search(t) or _MOVELIST.search(t):
        return plan("what_changed", room=room, since=first)
    if _C_TIME.search(t):
        return plan("timeline", since=first, room=room, limit=10)
    return None


def parse(text: str, known_places: list[str], picker=None) -> dict | None:
    """{"tool", "args"} for an incident question, {"reply": ...} for one that cannot be answered, None for anything else.
    Order: exact patterns (high precision) -> the LLM tool picker, when given (validated, may say none / fail) -> the cue-word classifier."""
    plan = _parse_exact(text, known_places)
    if plan is None and picker is not None:
        rooms = [p for p in known_places if re.fullmatch(r"room [A-Z]", p)]
        plan = picker.pick(text, rooms)
        if plan is not None and "style" in plan and plan.get("tool") == "object_history":
            return plan
    plan = plan or classify(text, known_places)
    if plan and plan.get("tool") == "object_history":
        if re.search(r"\bstill\b|\bstay(?:ed)?\s+(?:put|in\s+place|there|where)\b|\bsame\s+(?:place|spot)\b|\banymore\b|\bwhere\s+(?:it|they)\s+was\b|"
                     r"\bwhere\s+(?:i|you)\s+(?:left|put|saw)\s+(?:it|them)\b|\bin\s+(?:its|their)\s+(?:place|spot)\b", text, re.I):
            plan["style"] = {"still": True}                       # a yes / no question about staying: answered from the CURRENT graph state, Yes / No first
            if (rm := _room(" ".join(text.split()))):
                plan["style"]["room"] = rm
        elif re.search(r"\bwhere\b", text, re.I):
            plan["style"] = {"where": True}                       # "where did it end up": no Yes / No
    return plan


def _parse_exact(text: str, known_places: list[str]) -> dict | None:
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
        return plan("what_changed", room=room, since="away") if (_CHANGED.search(t) or _C_CHANGE.search(t) or _STATE.search(t)) else plan("timeline", since="away", room=room, limit=10)
    if _LAST_SEEN.search(t) and objs:
        return plan("last_seen", object=objs[0])
    if _HISTORY.search(t) and objs:
        return plan("object_history", object=objs[0], since=since)
    if _CHANGED.search(t):
        return plan("what_changed", room=room, since=since)
    if _TIMELINE.search(t):
        return plan("timeline", since=since, room=room, object=objs[0] if objs else None)
    return None
