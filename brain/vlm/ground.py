"""Grounding of VLM answers against the robot's own facts (docs/results/vlm_plan.md section 3). Pure Python.

facts: [{cls, color, x, y, source: detector | map}] = what the detector saw in THIS frame + semantic-map objects in the camera cone.
vlm:   the parsed VLM dict (client.parse) or None (unavailable / bad JSON).
The reply is built ONLY from templates + fact fields + checked VLM fields; the VLM's free `answer` is never used.

  * an object is stated as present only with a supporting fact
  * a VLM object of a detector class nobody detected -> "not sure"; a noun outside the detector's classes -> "can't confirm"
  * fact vs VLM disagreement (colour, count, path) -> "not sure", with both sides
  * `check_reply` (also used at runtime): every sentence that names an object without a fact must say not sure / can't confirm
"""
from __future__ import annotations

import math
import re
from collections import Counter
from dataclasses import dataclass, field

from .vocab import NOT_OBJECTS, SIBLINGS, WORDS, colour, noun, say, to_class

NEAR_M = 3.0          # "the box near the barrel": the nearest box within this distance of a barrel
TIE_M = 0.5           # ... and clearly nearer (by this margin) than the second-nearest box, else "which one?"
HEDGES = ("not sure", "can't confirm", "doesn't see", "don't see", "don't know", "can't tell", "can't check", "didn't",
          "sees no ", "see no ", "nothing")      # a hedge or a negative: the sentence does not claim the object is there


@dataclass
class Verdict:
    status: str                      # confirmed | not_sure | detector_only | not_found | ask | no_view | unknown
    reply: str
    confirmed: list = field(default_factory=list)       # facts the VLM also named
    detector_only: list = field(default_factory=list)   # facts the VLM did not name
    vlm_only: list = field(default_factory=list)        # [(kind unconfirmed | outside, word, colour)]
    conflicts: list = field(default_factory=list)       # [(fact, vlm colour)]
    vlm_used: bool = False
    notes: list = field(default_factory=list)


# ------------------------------------------------------------------ wording
def _a(words: str) -> str:
    return ("an " if words[:1] in "aeiou" else "a ") + words


def _desc(cls, col="") -> str:
    return f"{col + ' ' if col else ''}{say(cls)}"


def _plural(w: str) -> str:
    return w + ("es" if w.endswith(("x", "s", "ch")) else "s")


def _list(facts) -> str:
    c = Counter((f["cls"], colour(f.get("color")) or "") for f in facts)
    parts = []
    for (cls, col), n in c.items():
        d = _desc(cls, col)
        parts.append(_a(d) if n == 1 else f"{_num(n)} {_plural(d)}")
    return parts[0] if len(parts) == 1 else ", ".join(parts[:-1]) + " and " + parts[-1]


def _num(n: int) -> str:
    return ["no", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten"][n] if n <= 10 else str(n)


def _cap(s: str) -> str:
    return s[:1].upper() + s[1:]


# ------------------------------------------------------------------ matching
def match(vlm_objects, facts):
    """-> (confirmed [(fact, vlm colour)], conflicts [(fact, vlm colour)], vlm_only [(kind, word, colour)], unmatched facts)."""
    used, confirmed, conflicts, only = set(), [], [], []
    for o in vlm_objects or []:
        cls, col = to_class(o["name"])
        col = colour(o.get("color")) or col
        w = noun(o["name"])
        if cls is None:
            if w and w not in NOT_OBJECTS:
                only.append(("outside", w, col))
            continue
        cands = [i for i, f in enumerate(facts) if i not in used and (f["cls"] == cls or SIBLINGS.get(cls) == f["cls"])]
        if not cands:
            only.append(("unconfirmed", cls, col))
            continue
        cands.sort(key=lambda i: (facts[i]["cls"] != cls, not (col and colour(facts[i].get("color")) == col)))
        i = cands[0]
        used.add(i)
        fc = colour(facts[i].get("color"))
        (conflicts if col and fc and fc != col else confirmed).append((facts[i], col))
    return confirmed, conflicts, only, [f for i, f in enumerate(facts) if i not in used]


def _only_sentences(only) -> list:
    out = []
    unc = [(w, c) for k, w, c in only if k == "unconfirmed"]
    outside = [(w, c) for k, w, c in only if k == "outside"]
    if unc:
        names = " and ".join(_a(_desc(w, c)) for w, c in dict.fromkeys(unc))
        out.append(f"The camera model also mentions {names}, but my detector doesn't see that, so I'm not sure about it.")
    if outside:
        names = " and ".join(_a(f"{c + ' ' if c else ''}{w}") for w, c in dict.fromkeys(outside))
        out.append(f"It also mentions {names}, which my detector can't confirm.")
    return out


def _conflict_sentences(conflicts) -> list:
    return [f"I'm not sure about the colour of the {say(f['cls'])}: my detector says {colour(f.get('color'))}, the camera model says {c}."
            for f, c in conflicts]


def _vlm_note(vlm, why_missing):
    if vlm is None:
        return [f"(My camera model {why_missing}, so this is from my detector only.)"]
    if vlm.get("unsure"):
        return ["The camera model wasn't sure about this view."]
    return []


# ------------------------------------------------------------------ per question kind
def answer(q: dict, facts: list, vlm: dict | None, path: dict | None = None, why_missing: str = "isn't available") -> Verdict:
    k = q["kind"]
    if k == "path":
        return _path(q, vlm, path or {}, why_missing)
    if k == "see":
        return _see(facts, vlm, why_missing)
    if k == "colour":
        return _colour(q, facts, vlm, why_missing)
    if k == "count":
        return _count(q, facts, vlm, why_missing)
    if k == "on":
        return _on(q, facts, vlm, why_missing)
    return Verdict("unknown", "I can answer what I see, the colour of a thing, how many there are, and whether a way is clear.")


def _see(facts, vlm, why):
    if vlm is None or vlm.get("unsure"):
        if not facts:
            return Verdict("detector_only", "My detector doesn't see any object it knows here." + " " + " ".join(_vlm_note(vlm, why)))
        return Verdict("detector_only", f"I can see {_list(facts)}. " + " ".join(_vlm_note(vlm, why)), detector_only=list(facts))
    conf, confl, only, rest = match(vlm["objects"], facts)
    s = []
    if facts:
        s.append(f"I can see {_list(facts)}.")
    else:
        s.append("My detector doesn't see any object it knows here.")
    s += _conflict_sentences(confl) + _only_sentences(only)
    st = "confirmed" if not confl and not only else "not_sure"
    return Verdict(st, " ".join(s), [f for f, _ in conf], rest, only, confl, True)


def _resolve(q, facts):
    """-> (fact | None, verdict-if-unresolved | None)"""
    cls = q.get("cls")
    if cls is None:
        return None, Verdict("unknown", f"I don't know what a {noun(q.get('noun', 'thing')) or 'thing'} is, so I can't check its colour.")
    cands = [f for f in facts if f["cls"] == cls] or [f for f in facts if SIBLINGS.get(cls) == f["cls"]]
    what = f"the {say(cls)}" + (f" near the {say(q['near'])}" if q.get("near") else "")
    if q.get("near_noun") and q.get("near") is None:
        return None, Verdict("unknown", f"I don't know what a {noun(q['near_noun'])} is, so I can't find {what.split(' near')[0]} near it.")
    if not cands:
        return None, Verdict("not_found", f"I don't see a {say(cls)} here.")
    if q.get("near"):
        cands = [c for c in cands if c.get("x") is not None]
        refs = [f for f in facts if f["cls"] == q["near"] and f.get("x") is not None]
        if not cands:
            return None, Verdict("not_sure", f"I can see a {say(cls)}, but I don't know where it stands, so I can't tell which one is near the {say(q['near'])}.")
        if not refs:
            return None, Verdict("not_found", f"I don't see a {say(q['near'])} here.")
        dist = sorted(((min(math.hypot(c["x"] - r["x"], c["y"] - r["y"]) for r in refs), i) for i, c in enumerate(cands)))
        if dist[0][0] > NEAR_M:
            return None, Verdict("not_found", f"I don't see a {say(cls)} near the {say(q['near'])}.")
        if len(dist) > 1 and dist[1][0] - dist[0][0] < TIE_M:
            return None, Verdict("ask", f"I can see more than one {say(cls)} near the {say(q['near'])}: "
                                        f"{_list([cands[dist[0][1]], cands[dist[1][1]]])}. Which one?")
        return cands[dist[0][1]], None
    cols = {colour(c.get("color")) for c in cands}
    if len(cands) > 1 and len(cols) > 1:
        return None, Verdict("ask", f"I can see {_list(cands)}. Which one?")
    return cands[0], None


def _colour(q, facts, vlm, why):
    f, v = _resolve(q, facts)
    if v is not None:
        if vlm is not None:
            v.vlm_used = True
        return v
    what = f"the {say(q['cls'])}" + (f" near the {say(q['near'])}" if q.get("near") else "")
    fc = colour(f.get("color"))
    vc = ""
    if vlm is not None and not vlm.get("unsure"):
        same = [o for o in vlm["objects"] if to_class(o["name"])[0] in (q["cls"], SIBLINGS.get(q["cls"]))]
        cols = [colour(o.get("color")) or to_class(o["name"])[1] for o in same]
        cols = [c for c in cols if c]
        vc = fc if fc in cols else (cols[0] if cols else "")
    if fc and vc and fc == vc:
        return Verdict("confirmed", f"{_cap(what)} is {fc}.", [f], vlm_used=True)
    if fc and vc:
        return Verdict("not_sure", f"I'm not sure: my detector says {what} is {fc}, but the camera model says {vc}.", conflicts=[(f, vc)], vlm_used=True)
    if fc:
        tail = "the camera model didn't say" if vlm is not None and not vlm.get("unsure") else f"my camera model {why}" if vlm is None else "the camera model wasn't sure"
        return Verdict("detector_only", f"My detector says {what} is {fc} ({tail}).", detector_only=[f], vlm_used=vlm is not None)
    if vc:
        return Verdict("not_sure", f"{_cap(what)} looks {vc} to the camera model, but my detector has no colour for it, so I'm not sure.", vlm_used=True)
    return Verdict("not_sure", f"I can see {what}, but I can't tell its colour.", vlm_used=vlm is not None)


def _count(q, facts, vlm, why):
    cls = q.get("cls")
    if cls is None:
        return Verdict("unknown", f"I don't know what {_plural(noun(q.get('noun', 'thing')) or 'thing')} are, so I can't count them.")
    col = q.get("color") or ""
    n = sum(1 for f in facts if f["cls"] == cls and (not col or colour(f.get("color")) == col))
    word = _desc(cls, col)
    said = f"{_num(n)} {word if n == 1 else _plural(word)}"
    if vlm is None or vlm.get("unsure"):
        return Verdict("detector_only", f"My detector sees {said}. " + " ".join(_vlm_note(vlm, why)))
    m = sum(1 for o in vlm["objects"] if to_class(o["name"])[0] == cls and (not col or (colour(o.get("color")) or to_class(o["name"])[1]) == col))
    if m == n:
        return Verdict("confirmed", f"I can see {said}.", vlm_used=True)
    return Verdict("not_sure", f"I'm not sure how many: my detector sees {said}, the camera model counts {_num(m)}.", vlm_used=True)


def _on(q, facts, vlm, why):
    surf, scls = q["surface"], q.get("surface_cls")
    rel = q["relation"]
    s = []
    if scls is None:
        w = noun(surf)
        mentioned = vlm is not None and any(noun(o["name"]) == w for o in vlm["objects"])
        s.append(f"I can't confirm a {w}: my detector doesn't know {_plural(w)}" + (", though the camera model mentions one." if mentioned else "."))
        if facts:
            s.append(f"What my detector sees here: {_list(facts)}.")
        if vlm is not None and not vlm.get("unsure"):
            conf, confl, only, rest = match(vlm["objects"], facts)
            on_it = [f for f, c in conf if any(noun(o["name"]) and (o.get("where") or "").lower().find(w) >= 0 and to_class(o["name"])[0] in (f["cls"], SIBLINGS.get(f["cls"]))
                                               for o in vlm["objects"])]
            if on_it:
                s.append(f"The camera model says {_list(on_it)} {'is' if len(on_it) == 1 else 'are'} {rel} it, which I can't check.")
            s += _only_sentences([x for x in only if not (x[0] == "outside" and x[1] == w)])
        else:
            s += _vlm_note(vlm, why)
        return Verdict("not_sure", " ".join(s), vlm_used=vlm is not None)
    base = [f for f in facts if f["cls"] == scls]
    if not base:
        return Verdict("not_found", f"I don't see a {say(scls)} here.")
    b = base[0]
    near = [f for f in facts if f is not b and None not in (f.get("x"), b.get("x")) and math.hypot(f["x"] - b["x"], f["y"] - b["y"]) <= 0.6]
    if near:
        s.append(f"At the {say(scls)} I can see {_list(near)} (I can't tell whether {'it is' if len(near) == 1 else 'they are'} {rel} it or beside it).")
    else:
        s.append(f"My detector sees nothing at the {say(scls)}.")
    if vlm is not None and not vlm.get("unsure"):
        conf, confl, only, rest = match(vlm["objects"], [f for f in facts if f is not b])
        s += _conflict_sentences(confl) + _only_sentences(only)
        return Verdict("confirmed" if not confl and not only else "not_sure", " ".join(s), [f for f, _ in conf], vlm_used=True)
    return Verdict("detector_only", " ".join(s + _vlm_note(vlm, why)))


def _path(q, vlm, p, why):
    if p.get("unknown_place"):
        return Verdict("unknown", f"I don't know a place called {p['unknown_place']}.")
    where = f"the way to {p['label']}" if p.get("label") else "the way ahead"
    reach = p.get("reachable")
    if reach is None:
        return Verdict("not_sure", f"I can't check {where} right now.")
    geo = f"my path check finds a way to {p['label']}" if reach and p.get("label") else ("nothing is in my way ahead" if reach else
          f"my path check finds no way through ({p.get('blocked_by') or 'blocked'})")
    if not p.get("in_view"):
        st = "detector_only"
        r = f"{'Yes' if reach else 'No'}, {where} is {'clear' if reach else 'blocked'}: {geo}. It isn't in front of me, so I checked it on the map only."
        return Verdict(st, r)
    if vlm is None or vlm.get("unsure") or vlm.get("path_clear") is None:
        return Verdict("detector_only", f"{'Yes' if reach else 'No'}, {where} is {'clear' if reach else 'blocked'}: {geo}. " + " ".join(_vlm_note(vlm, why) or
                       ["The camera model didn't say."]))
    if vlm["path_clear"] == bool(reach):
        return Verdict("confirmed", f"{'Yes' if reach else 'No'}, {where} {'looks clear' if reach else 'is blocked'}: {geo}, and the camera agrees.", vlm_used=True)
    return Verdict("not_sure", f"I'm not sure about {where}: {geo}, but the camera model thinks it is {'blocked' if reach else 'clear'}.", vlm_used=True)


# ------------------------------------------------------------------ the invariant
_OBJ_WORDS = sorted(WORDS, key=len, reverse=True)


def check_reply(reply: str, facts: list, extra_nouns=()) -> list:
    """Sentences that name an object (a class word, or a VLM-only noun in `extra_nouns`) with no supporting fact and no hedge.
    [] = the reply only states facts (grounded)."""
    have = {f["cls"] for f in facts} | {SIBLINGS.get(f["cls"]) for f in facts}
    bad = []
    for sent in re.split(r"(?<=[.?!])\s+", reply):
        low = re.sub(r"\bcan(?:'t|not)?(?=\s+(?:see|tell|confirm|check|answer|be|count|find|look|only)\b)", " ", sent.lower())   # the verb, not the class
        if any(h in low for h in HEDGES):
            continue
        named = {WORDS[w] for w in _OBJ_WORDS if re.search(rf"\b{re.escape(w)}\b", low)} | {"cube" if "cube" in low else None}
        named.discard(None)
        named = {("small_box" if n == "cube" else n) for n in named}
        if named - have or any(re.search(rf"\b{re.escape(n)}\b", low) for n in extra_nouns):
            bad.append(sent)
    return bad
