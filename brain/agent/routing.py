"""Deterministic routing of object-search plans (held-out lab4 failures, docs/results/phase5_heldout_lab4.md; fixes in phase5_heldout_lab4_v2.md).

The planner is untrusted text. After it answered and before the guard / safety checker, `route_plan` repairs the TOOL CHOICE for requests that
name an object and ask the robot to go and look for it:

  * `where_is` (a memory lookup) for "go and check where the yellow box is"      -> `find` (it looks, and answers from memory if it already knows)
  * `explore` without any search for "look around for the mannequin"             -> `find` (explore ends at once on a mapped room and the robot said "Done.")
  * `find` / `where_is` with an invented argument (`place`, `area`, `room` ...)   -> the argument is removed; a KNOWN place becomes a `go_to` first
                                                                                   ("find the cone in room D" = go there, then look), an UNKNOWN place is
                                                                                   refused with the list of known places (never guessed)

A pure memory question ("where is the red box?", "where can I find the toolbox?") keeps `where_is`: it answers from memory and never moves.
No LLM, no state: regexes over the user's words (exact class words and colours only, no fuzzy matching: "charlie" is not "chair").
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field

from ..tools import TOOLS

# an ACTION cue: the user wants the robot to look / go and look (not just to recall what it knows)
_ACTION = re.compile(
    r"\b(find|look(?:ing)?\s+(?:around\s+)?for|search(?:ing)?(?:\s+for)?|locate|track\s+down|hunt(?:\s+for)?|seek|scout(?:\s+for)?|spot|"
    r"go\s+(?:and\s+)?(?:check|look|see|find)|head\s+(?:over\s+)?(?:and|to)\s+(?:check|look)|check\s+(?:where|if|whether|for|out)|see\s+if|"
    r"have\s+a\s+look\s+for|keep\s+an\s+eye\s+out\s+for|make\s+sure\s+where|verify\s+where|confirm\s+where)\b", re.I)
# "where can I find X" is a question about what the robot knows, not an order to search
_QUESTION_FIND = re.compile(r"\bwhere\s+(?:can|could|do|would|might|should)\s+(?:i|we|you)\s+find\b", re.I)
_GO_CUE = re.compile(r"\b(go|head|walk|drive|approach|take me|bring me|come|move)\b", re.I)
_COLOURS = ("red", "blue", "yellow", "green", "white", "orange", "black", "brown", "grey", "gray")
_PLACE_KEYS = ("place", "area", "room", "location", "where", "in", "near")
# "find the cone IN room D": the search is restricted to that room (rooms.normalize has already turned "Delta" into "room D")
_IN_ROOM = re.compile(r"\b(?:in|inside|within|around|at)\s+(?:the\s+)?(room\s+[a-z])\b", re.I)

# exact class words (plural too); NOT fuzzy: "charlie" is not "chair"
_WORD_TO_CLASS = {"box": "box", "boxes": "box", "crate": "box", "crates": "box", "barrel": "barrel", "barrels": "barrel", "drum": "barrel",
                  "extinguisher": "fire_extinguisher", "extinguishers": "fire_extinguisher", "cone": "traffic_cone", "cones": "traffic_cone",
                  "toolbox": "toolbox", "toolboxes": "toolbox", "mannequin": "mannequin", "mannequins": "mannequin", "dummy": "mannequin",
                  "chair": "chair", "chairs": "chair"}


@dataclass
class Route:
    calls: list[dict]
    reply: str | None = None
    rule: str = ""
    changed: bool = False


def _tokens(text: str) -> list[str]:
    return re.findall(r"[a-z']+", text.lower())


def named_objects(text: str) -> list[dict]:
    """Object references the user NAMED (exact class words, optional colour right before): [{"class": ..., "color": ...?}]. Distinct classes only."""
    toks = _tokens(text)
    out: list[dict] = []
    for i, t in enumerate(toks):
        cls = _WORD_TO_CLASS.get(t)
        if cls is None or any(o["class"] == cls for o in out):
            continue
        ref: dict = {"class": cls}
        for back in (1, 2):
            if i - back >= 0 and toks[i - back] in _COLOURS:
                ref["color"] = "grey" if toks[i - back] == "gray" else toks[i - back]
                break
        out.append(ref)
    return out


def wants_search(text: str) -> bool:
    """The user asked the robot to look for something (not just where it remembers it)."""
    return bool(_ACTION.search(_QUESTION_FIND.sub(" ", text)))


def _canon(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", " ", str(name).lower()).strip()


def _known(name: str, places: list[str]) -> str | None:
    n = _canon(name)
    return next((p for p in places if _canon(p) == n), None)


def place_in_utterance(text: str, places: list[str]) -> str | None:
    """The known room the user wants the search restricted to ("... in room D"), else None."""
    m = _IN_ROOM.search(text)
    return _known(m.group(1), places) if m else None


def _schema_keys(tool: str) -> set[str]:
    return set(TOOLS[tool].parameters.get("properties", {})) if tool in TOOLS else set()


def route_plan(utterance: str, calls: list[dict], known_places: list[str]) -> Route:
    """Repair the tool choice of a plan (see the module doc). `calls` are the planner's calls, returned unchanged if nothing applies."""
    if not calls or any(not isinstance(c, dict) or not isinstance(c.get("tool"), str) for c in calls):
        return Route(calls)
    out: list[dict] = []
    rules: list[str] = []
    search = wants_search(utterance)
    objs = named_objects(utterance)
    have_find = any(c["tool"] == "find" for c in calls)

    for c in calls:
        tool = c["tool"]
        args = dict(c["args"]) if isinstance(c.get("args"), dict) else {}
        # ---- an argument the tool does not have, naming a place ("find ... place: room D")
        if tool == "find" and "in_place" in args:             # internal argument: only the router sets it, never the planner
            claimed = args.pop("in_place")
            if isinstance(claimed, str) and _known(claimed, known_places) and "place" not in args:
                args["place"] = claimed
        extra = [k for k in args if k in _PLACE_KEYS and k not in _schema_keys(tool)] if tool in ("find", "where_is", "inspect") else []
        for k in extra:
            val = args.pop(k)
            if not isinstance(val, str):
                continue
            real = _known(val, known_places)
            if real is None:
                return Route([], reply=f"I can't do that. I do not know a place called {val}. I know: {', '.join(known_places)}.",
                             rule="route_unknown_place", changed=True)
            if tool == "find":
                args["in_place"] = real                       # only an object seen IN that place counts; if none is known there, look there
                rules.append("place_arg->in_place")
            elif tool == "inspect":
                out.append({"tool": "go_to", "args": {"place": real}})
                rules.append("place_arg->go_to")
            else:
                rules.append("place_arg_dropped")
        # ---- memory lookup although the user asked the robot to go and look
        if tool == "where_is" and search and isinstance(args.get("object"), dict):
            if not have_find:
                new = {"object": args["object"]}
                if _GO_CUE.search(utterance):
                    new["approach"] = True
                if (ip := place_in_utterance(utterance, known_places)):
                    new["in_place"] = ip
                out.append({"tool": "find", "args": new})
                have_find = True
                rules.append("where_is->find")
            else:
                rules.append("where_is_dropped")
            continue
        # ---- explore although a named object is the point of the request
        if tool == "explore" and search and len(objs) == 1 and not have_find:
            new = {"object": objs[0]}
            if (ip := place_in_utterance(utterance, known_places)):
                new["in_place"] = ip
            out.append({"tool": "find", "args": new})
            have_find = True
            rules.append("explore->find")
            continue
        if tool == "find" and "in_place" not in args and (ip := place_in_utterance(utterance, known_places)) and isinstance(args.get("object"), dict):
            args["in_place"] = ip                             # the planner dropped "in room D" (or never had it): the user's words decide
            rules.append("find_in_place")
        out.append({"tool": tool, "args": args})
    return Route(out, rule=",".join(rules), changed=bool(rules))
