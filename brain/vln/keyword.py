"""Keyword fallback for VLN (no LLM): instruction -> the SAME constrained JSON the model would return (brain/vln/plan.py),
validated by the same code. Used when the model is unreachable, returns bad JSON, or its plan is rejected; also alone when
the stack runs without an LLM (MODEL=keyword).

Deterministic and deliberately narrow. It returns None (the normal planner / unknown-noun guard handles the text) whenever it
is not sure, in particular:
  * a noun after "the" that is neither a detector class, a colour nor a room name ("go past the sofa"): an unknown landmark
    is never silently dropped from the route
  * order words that reverse the spoken order ("before", "after <x>"); "first", "then", "afterwards" keep it
  * a route that does not end at a place (a lone "go past the chair")
Clauses are split on , ; "then" "and"; "between X and Y" and "avoid room B and room C" are taken out first.
"""
from __future__ import annotations

import re

from .plan import COLOURS, WORDS, _room_word_re, avoid_re, mentions, rooms_said

RELATIONS = (("left_of", r"(?:to |on )?the left(?: side)? of|left of"), ("right_of", r"(?:to |on )?the right(?: side)? of|right of"),
             ("in_front_of", r"in front of"), ("behind", r"behind|in back of"), ("next_to", r"next to|beside|alongside"))
PASS_RE = re.compile(r"\b(?:go(?:ing)? past|past|passing|by|via|through|thru|around)\b", re.I)
ROOM_WITH_RE = re.compile(r"\broom (?:with|that has|containing|where)\b", re.I)
ORDER_RE = re.compile(r"\b(?:before|after(?! that\b))\b", re.I)     # these reverse the spoken order ("first ... then" keeps it)
FILLER = {"room", "left", "right", "spot", "middle", "front", "back", "side", "way", "end", "other", "place", "area", "one",
          "same", "corner", "door", "doorway", "centre", "center", "robot", "floor", "first", "second", "next", "last", "big", "small",
          "little", "large"}


def _unknown_noun(s: str, names: dict) -> str | None:
    for m in re.finditer(r"\bthe\s+([a-z]+)(?:\s+([a-z]+))?", s):
        w, w2 = m.group(1), m.group(2)
        if w in COLOURS or w in FILLER:
            if w in COLOURS and w2 and w2 not in WORDS and w2 not in names and w2 not in FILLER:
                return w2                                       # "the green sofa"
            continue
        if w in WORDS or w in names:
            continue
        if any(n.startswith(w + " ") for n in names):            # a two-word room name ("living room")
            continue
        return w
    return None


LANDMARK_POS = re.compile(r"\b(?:past|passing|by|near|to|at|towards?|toward|into|through|via|around|behind|beside|of|between|and)\s+the\s+"
                          r"(?:(?:" + "|".join(COLOURS) + r")\s+)?([a-z]+)\b")


def unknown_landmark(text: str, world, known_places=()) -> str | None:
    """A word in a landmark position ("past the sofa", "near the green sofa") that is no detector class, no room name and no
    known place: VLN answers that it does not know it (never walks a route with it dropped)."""
    t = text.lower()
    names = world.names or {}
    places = [p.lower() for p in known_places]
    for m in LANDMARK_POS.finditer(t):
        w = m.group(1)
        if w in WORDS or w in names or w in FILLER or w in COLOURS or any(n.startswith(w + " ") for n in names):
            continue
        if any(p == w or p.startswith(w + " ") or p.startswith("the " + w) for p in places):
            continue
        return w
    return None


def _ref(clause: str, world, idx: int = 0):
    m = mentions(clause)
    if len(m) <= idx:
        return None
    r = dict(m[idx])
    hits = [w for w in re.finditer(r"[a-z]+", clause) if w.group(0) in WORDS]
    rest = clause[hits[idx].end():] if len(hits) > idx else ""
    if len(hits) > idx + 1:                                     # only up to the next object word
        rest = clause[hits[idx].end():hits[idx + 1].start()]
    inr = re.search(r"\bin\s+(" + _room_word_re(world.names) + ")", rest)
    if inr:
        rr = rooms_said(inr.group(1), world)
        if rr:
            r["in_room"] = rr[0]
    return r


def parse(text: str, world) -> dict | None:
    """-> {"kind": "go", "steps": [...], "avoid": [...]} or None (not sure: the normal planner handles the text)."""
    t = " " + re.sub(r"[^a-z0-9,;' ]+", " ", text.lower()) + " "
    names = world.names or {}
    if ORDER_RE.search(t):
        return None
    avoid = []
    for m in avoid_re(names).finditer(t):
        avoid += rooms_said(m.group(1), world)
    t = avoid_re(names).sub(" , ", t)
    t = re.sub(r"\bbut\b", " , ", t)
    t = re.sub(r"\b(stop|wait|end up|park|stand|halt)\s+by\b", r"\1 near", t)       # "stop by X" = stop near X, not "pass"
    if _unknown_noun(t, names) is not None:
        return None
    pieces, rest = [], t
    for m in re.finditer(r"\bbetween\b(.+?)\band\b(.+?)(?=,|;|\bthen\b|$)", t):
        pieces.append(("between", m.group(0)))
    rest = re.sub(r"\bbetween\b(.+?)\band\b(.+?)(?=,|;|\bthen\b|$)", " , @between@ , ", t)
    clauses = []
    for c in re.split(r",|;|\band then\b|\bthen\b|\band\b|\bafter that\b|\bafterwards\b|\bnext\b(?! to\b)|\bfirst\b", rest):
        # one target per segment: "go to the kitchen past the chair" -> "go to the kitchen" | "past the chair";
        # "past the blue box into room D" -> "past the blue box" | "into room D" ("next to" / "close to" stay whole)
        clauses += [x.strip() for x in re.split(r"(?=\b(?:go(?:ing)? past|past|passing|via|through|thru)\b)|(?<!next )(?<!close )(?<!near )(?<!up )(?=\b(?:into|to|towards?|toward|onto)\b)", c)
                    if x.strip()]
    steps, bi = [], iter(pieces)
    for c in clauses:
        if c == "@between@":
            seg = next(bi)[1]
            a, b = _ref(seg, world, 0), _ref(seg, world, 1)
            if a is None or b is None:
                return None
            steps.append({"action": "stop", "target": {"relation": "between", "of": [a, b]}})
            continue
        objs = mentions(c)
        rms = [] if objs and re.search(r"\bin\s+" + _room_word_re(names), c) and not re.search(
            r"\b(?:to|into|enter|through|via|past)\s+" + _room_word_re(names), c) else rooms_said(c, world)
        rel = next((name for name, pat in RELATIONS if re.search(rf"\b(?:{pat})\b", c)), None)
        if rel is not None:
            r = _ref(c, world)
            if r is None:
                return None
            steps.append({"action": "stop", "target": {"relation": rel, "of": [r]}})
        elif ROOM_WITH_RE.search(c):
            r = _ref(c.split(ROOM_WITH_RE.search(c).group(0), 1)[1], world)
            if r is None:
                return None
            steps.append({"action": "enter", "target": {"room_with": r}})
        elif PASS_RE.search(c) and (objs or rms):
            pos_obj = c.find(objs[0]["class"].split("_")[-1]) if objs else 10 ** 6
            first_room = re.search(_room_word_re(names), c)
            if rms and (not objs or (first_room and first_room.start() < pos_obj)):
                steps.append({"action": "pass", "target": {"room": rms[0]}})
            else:
                steps.append({"action": "pass", "target": {"object": _ref(c, world)}})
        elif rms:
            steps.append({"action": "enter", "target": {"room": rms[0]}})
        elif objs:
            steps.append({"action": "stop", "target": {"object": _ref(c, world)}})
        # a clause with no place ("go", "please", "now") adds nothing
    if not steps:
        return None
    # never drop what the user named: every object word and every room word (outside the avoid phrases) is in the plan
    refs = [r for s_ in steps for r in ([s_["target"].get("object"), s_["target"].get("room_with")] + s_["target"].get("of", [])) if r]
    if len(refs) != len(mentions(t)):
        return None
    rooms_used = [s_["target"]["room"] for s_ in steps if "room" in s_["target"]] + [r["in_room"] for r in refs if r.get("in_room")]
    if len(rooms_used) != len(rooms_said(t, world)):
        return None
    for s in steps[:-1]:                                          # "go to the red box, then room D": visit = pass on the way
        if s["action"] == "stop":
            if "relation" in s["target"]:
                return None
            s["action"] = "pass"
    last = steps[-1]
    if last["action"] == "pass":
        return None                                               # the walk must end somewhere the user said
    if len(steps) > 6:
        return None
    return {"kind": "go", "steps": steps, "avoid": sorted(set(avoid))}
