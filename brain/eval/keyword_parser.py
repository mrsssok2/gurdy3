"""Keyword-parser baseline (Gurdy-1.0 style): regex rules, no learning, no context. This is the system the LLM
has to beat in the evaluation (5.3). It understands one phrase per rule and never asks for clarification."""
from __future__ import annotations

import re

try:
    from gurdy_core.vision.classes import ALIASES
except ImportError:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "gurdy_core"))
    from gurdy_core.vision.classes import ALIASES

_COLOURS = ("red", "green", "blue", "yellow", "orange", "white", "black", "grey", "gray")
_NUM = r"(\d+(?:\.\d+)?)"
_STOP = re.compile(r"\b(stop|halt|freeze|emergency)\b")
_DIRS = {"forward": "forward", "forwards": "forward", "ahead": "forward", "straight": "forward", "back": "backward",
         "backward": "backward", "backwards": "backward", "left": "left", "right": "right"}


def _obj(text: str) -> dict | None:
    words = re.findall(r"[a-z]+", text.lower())
    cls = next((c for c, al in ALIASES.items() if any(w in al for w in words)), None)
    if cls is None:
        return None
    # "fire"/"tool"/"traffic" alone are weak aliases; accept them like the old parser did
    o = {"class": cls}
    col = next((w for w in words if w in _COLOURS), None)
    if col:
        o["color"] = "grey" if col == "gray" else col
    return o


def _place(text: str, places: list[str]) -> str | None:
    t = " " + re.sub(r"[^a-z0-9 ]", " ", text.lower()) + " "
    for p in sorted(places, key=len, reverse=True):
        if f" {p.lower()} " in t:
            return p
    return None


def _parse_clause(c: str, places: list[str]) -> dict | None:
    c = c.strip().lower().rstrip(".?!")
    if not c:
        return None
    if _STOP.search(c):
        return {"tool": "stop", "args": {}}
    m = re.search(r"\b(pause|resume|continue|cancel|abort)\b", c)
    if m:
        a = {"continue": "resume", "abort": "cancel"}.get(m.group(1), m.group(1))
        return {"tool": "mission_control", "args": {"action": a}}
    if re.search(r"no[- ]?go", c):
        act = "remove" if re.search(r"\b(remove|delete|clear)\b", c) else ("list" if re.search(r"\b(list|which|show)\b", c) else "add")
        args: dict = {"action": act}
        p = _place(c, places)
        if p and act != "list":
            args["place"] = p
        return {"tool": "no_go_zone", "args": args}
    m = re.search(r"\bsave\b.*\b(?:as|called|named)\s+([a-z0-9 ]+)$", c)
    if m:
        return {"tool": "save_place", "args": {"name": m.group(1).strip()}}
    m = re.search(r"\bturn\b(?:\s+(?:to the|to))?\s*(left|right|around)?(?:.*?" + _NUM + r"\s*(?:degrees?|deg|°))?", c)
    if m and re.search(r"\bturn\b", c):
        d = m.group(1) or "left"
        ang = float(m.group(2)) if m.group(2) else (180.0 if d == "around" else 90.0)
        return {"tool": "turn", "args": {"direction": "left" if d == "around" else d, "angle_deg": ang}}
    m = re.search(r"\b(?:walk|move|go|step)\b\s+(?:to the\s+)?(forward|forwards|ahead|straight|back|backward|backwards|left|right)\b"
                  r"(?:.*?" + _NUM + r"\s*(metres?|meters?|m|cm|centimet\w+|steps?))?", c)
    if m:
        dist = 1.0
        if m.group(2):
            n, u = float(m.group(2)), m.group(3)
            dist = n / 100 if u.startswith("c") else (n * 0.3 if u.startswith("step") else n)
        return {"tool": "walk", "args": {"direction": _DIRS[m.group(1)], "distance_m": min(3.0, max(0.1, round(dist, 2)))}}
    if re.search(r"\b(go|come|return|head|get)\b.*\b(home|back)\b|\bgo home\b", c) and not _obj(c):
        return {"tool": "return_home", "args": {}}
    if re.search(r"\bexplore\b", c):
        return {"tool": "explore", "args": {}}
    if re.search(r"\bpatrol\b", c):
        ps = [p for p in places if p.lower() in c] or ["all"]
        return {"tool": "patrol", "args": {"places": ps}}
    if re.search(r"\b(find|look for|search for|locate)\b", c):
        o = _obj(c)
        return {"tool": "find", "args": {"object": o, "approach": bool(re.search(r"\bgo\b|\bapproach\b", c))}} if o else None
    if re.search(r"\binspect\b|\bcheck on\b", c):
        o = _obj(c)
        return {"tool": "inspect", "args": {"object": o}} if o else None
    if re.search(r"\b(go|navigate|head|drive|walk|come)\b.*\b(to|towards|toward)\b", c):
        p = _place(c, places)
        if p:
            return {"tool": "go_to", "args": {"place": p}}
        o = _obj(c)
        if o:
            return {"tool": "go_to", "args": {"object": o}}
        return None
    if re.search(r"\bwhere (?:is|are)\b.*\b(you|i)\b|\bwhere am i\b|\bwhere are you\b", c):
        return {"tool": "where_am_i", "args": {}}
    if re.search(r"\bwhere\b.*\b(is|are|was)\b", c):
        o = _obj(c)
        return {"tool": "where_is", "args": {"object": o}} if o else None
    if re.search(r"\b(nearby|around you|close to you|near you)\b", c):
        return {"tool": "whats_nearby", "args": {}}
    if re.search(r"\b(battery|status|how are you|legs?)\b", c):
        return {"tool": "status", "args": {}}
    if re.search(r"\bwhy\b", c):
        return {"tool": "why", "args": {"about": "stop" if "stop" in c else "last_task"}}
    if re.search(r"\b(what do you see|what can you see|describe)\b", c):
        return {"tool": "describe_view", "args": {}}
    if re.search(r"\breport\b|\bsummary\b", c):
        return {"tool": "report", "args": {}}
    return None


# Repeat counts and delays (rt24 / rt29, 2026-11-27): the parser used to DROP "100000 times" and the "schedule ... in N s"
# wrapper and run the command once, now. A count or delay it cannot carry over exactly is never dropped: it is put into the
# call (patrol loops, schedule delay_s) when the tool can hold it, else the parser asks / refuses instead of acting.
_COUNT = re.compile(r"\b(\d[\d,]*(?:\.\d+)?)\s*(?:x\s+|times?\b|loops?\b|rounds?\b|laps?\b)|\b(once|twice|thrice)\b")
_DELAY = re.compile(r"\b(?:in|after)\s+(\d[\d,]*(?:\.\d+)?)\s*(seconds?|secs?|minutes?|mins?|hours?|hrs?|days?)\b")
_SCHEDULE = re.compile(r"\b(schedule|later|remind)\b")
_UNIT_S = {"s": 1, "m": 60, "h": 3600, "d": 86400}
MAX_LOOPS, MAX_DELAY_S = 10, 86400            # = the patrol / schedule tool limits (brain/tools/specs.py)


def _num(s: str) -> float:
    return float(s.replace(",", ""))


def parse(text: str, places: list[str]) -> dict:
    """-> {"calls": [...]} or {"reply": "I did not understand."} like the LLM planner contract."""
    low = text.lower()
    d = _DELAY.search(low)
    if d or _SCHEDULE.search(low):
        if not d:
            return {"reply": "When should I do that? Tell me the delay, for example 'in 5 minutes'."}
        delay = _num(d.group(1)) * _UNIT_S[d.group(2)[0]]
        if delay > MAX_DELAY_S:
            return {"reply": "I can only schedule something up to 24 hours ahead, so I won't schedule that."}
        if delay < 1:
            return {"reply": "When should I do that? Tell me the delay, for example 'in 5 minutes'."}
        rest = _SCHEDULE.sub(" ", low[:d.start()] + " " + low[d.end():])
        rest = re.sub(r"^\s*(?:a|to)\b", " ", rest)
        inner = _parse_plain(rest, places)
        if "reply" in inner:
            return inner
        if len(inner["calls"]) != 1 or inner["calls"][0]["tool"] in ("stop", "mission_control", "schedule"):
            return {"reply": "I can only schedule one command at a time. Which one should I do later?"}
        return {"calls": [{"tool": "schedule", "args": {"delay_s": int(round(delay)), "call": inner["calls"][0]}}], "continue": False}
    return _parse_plain(low, places)


def _parse_plain(low: str, places: list[str]) -> dict:
    c = _COUNT.search(low)
    n = None
    if c:
        n = {"once": 1, "twice": 2, "thrice": 3}[c.group(2)] if c.group(2) else _num(c.group(1))
        low = low[:c.start()] + " " + low[c.end():]
    out = _parse_clauses(low, places)
    if n is None or "reply" in out:
        return out
    pat = [k for k in out["calls"] if k["tool"] == "patrol"]
    if pat and len(out["calls"]) == 1:
        if n > MAX_LOOPS or n != int(n) or n < 1:
            return {"reply": f"I can patrol at most {MAX_LOOPS} loops in one go, so I won't start {n:g}. How many loops should I do?"}
        if n > 1:
            pat[0]["args"]["loops"] = int(n)
        return out
    if n == 1:
        return out
    return {"reply": f"I can't repeat that {n:g} times. Ask me once, or tell me what to do step by step."}


def _parse_clauses(low: str, places: list[str]) -> dict:
    clauses = [low] if "patrol" in low else re.split(r"\b(?:and then|then|after that|and)\b|[,;]", low)
    calls = []
    for cl in clauses:
        if not cl.strip():
            continue
        call = _parse_clause(cl, places)
        if call is None:
            return {"reply": "Sorry, I did not understand that."}
        calls.append(call)
    return {"calls": calls, "continue": False} if calls else {"reply": "Sorry, I did not understand that."}
