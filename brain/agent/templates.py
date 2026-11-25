"""Template phrasing built directly from tool results (every number / name comes from the result, so it is
grounded by construction). Used for motion acknowledgements, event announcements and the grounding fallback."""
from __future__ import annotations

import re

from typing import Any


def _cls(c: str) -> str:
    return c.replace("_", " ")


def obj_name(o: dict) -> str:
    return f"{o.get('color') or ''} {_cls(o['class'])}".strip()


def a_obj(o: dict) -> str:
    n = obj_name(o)
    return ("an " if n[:1] in "aeiou" else "a ") + n


def obj_where(o: dict) -> str:
    return f"{obj_name(o)} in {o['place']}, {o['distance_m']:.1f} metres {o['bearing_word']}"


def _target_name(a: dict) -> str:
    if a.get("place"):
        from .rooms import canon_place
        return canon_place(a["place"])
    if a.get("object"):
        return obj_name(a["object"])
    if a.get("object_id"):
        return f"object {a['object_id']}"
    if a.get("point"):
        return "that point"
    return "the target"


FRIENDLY_ERRORS = {"INVALID_ARGS": "I could not work out how to do that. Could you say it another way?",
                   "UNKNOWN_TOOL": "I can't do that. I don't have a way to do it."}


def error_text(res: dict) -> str:
    """User-facing text of a failed tool result: no validator internals."""
    err = res["error"]
    return FRIENDLY_ERRORS.get(err.get("code"), err["message"])


def ack(tool: str, args: dict, data: dict) -> str:
    """Spoken acknowledgement of a started motion tool, e.g. 'Going to room B, 4.2 metres.'"""
    est = data.get("estimate", {})
    dist = f", {est['distance_m']:.1f} metres" if est.get("distance_m") else ""
    if tool == "walk":
        return f"Walking {args['direction']} {args['distance_m']:g} metres."
    if tool == "turn":
        return f"Turning {args['direction']} {args['angle_deg']:g} degrees."
    if tool == "go_to":
        return f"Going to {_target_name(args)}{dist}."
    if tool == "return_home":
        return f"Going home{dist}."
    if tool == "find":
        return f"Looking for the {obj_name(args['object'])}{' in ' + rooms_canon(args['in_place']) if args.get('in_place') else ''}."
    if tool == "explore":
        return "Exploring."
    if tool == "patrol":
        return "Starting the patrol."
    if tool == "inspect":
        return f"Inspecting {_target_name(args)}{dist}."
    if tool == "stop":
        return "Stopped."
    return "Okay."


def rooms_canon(name: str) -> str:
    from .rooms import canon_place
    return canon_place(name)


def readback(tool: str, args: dict) -> str:
    """What a low-confidence command is read back as, then "Shall I go?": 'Room A, Alpha' / 'The red box' / 'Walking forward 1 metres'."""
    from .rooms import phonetic
    place = args.get("place", "")
    if tool in ("go_to", "inspect") and re.fullmatch(r"room [A-Z]", place):
        return f"Room {place[-1]}, {phonetic(place[-1]) or place[-1]}"
    if tool == "go_to" and place:
        return place[0].upper() + place[1:]
    if tool == "go_to" and args.get("object"):
        return "The " + obj_name(args["object"])
    if tool == "return_home":
        return "Go home"
    return ack(tool, args, {}).rstrip(".")


def preview(tool: str, args: dict) -> str:
    """A pending step named in the plan, WITHOUT parameters the planner filled in (directions, distances, angles are not in any tool result
    yet, so the grounding rule forbids stating them): 'go to room B', 'look for the red box', 'walk', 'turn'."""
    if tool == "go_to":
        return f"go to {_target_name(args)}"
    if tool == "find":
        return f"look for the {obj_name(args['object'])}"
    if tool == "inspect":
        return f"inspect {_target_name(args)}"
    return {"return_home": "go home", "explore": "explore", "patrol": "patrol", "walk": "walk", "turn": "turn"}.get(tool, tool.replace("_", " "))


def question_for(tool: str) -> str:
    return "Shall I go?" if tool in ("go_to", "return_home", "inspect", "find", "explore", "patrol") else "Shall I do that?"


def _age(s: float) -> str:
    if s < 90:
        return "just now"
    m = s / 60
    return f"{m:.0f} minutes ago" if m < 90 else f"{m / 60:.0f} hours ago"


def _record(r: dict) -> str:
    """One recall record: object (with age when it comes from memory), mission (tool, target, outcome) or place."""
    if "class" in r:
        age = f", last seen {_age(r['age_s'])}" if r.get("from_memory") else ""
        stale = " (may be out of date)" if r.get("stale") else ""
        return f"{obj_name(r)} in {r['place']}{age}{stale}"
    if "tool" in r and "state" in r:
        tgt = f" {r['target']}" if r.get("target") else ""
        return f"{r['tool'].replace('_', ' ')}{tgt}, {r['state']}, {_age(r['age_s'])}"
    return str(r.get("name", r.get("tool")))


def summarize(tool: str, args: dict, res: dict) -> str:
    """One sentence for a tool result (ok or error)."""
    if not res["ok"]:
        return error_text(res)
    d = res["data"]
    if tool == "where_is":
        return "; ".join(f"The {obj_where(o)}" for o in d["matches"]) + "."
    if tool == "whats_nearby":
        parts = [f"{a_obj(o)}, {o['distance_m']:.1f} metres {o['bearing_word']}" for o in d["objects"]]
        obs = [f"obstacle {o['bearing_word']}, {o['distance_m']:.1f} metres" for o in d["obstacles"]]
        items = parts + obs
        return f"In {d['place']}: " + ("; ".join(items) if items else "nothing close by") + "."
    if tool == "where_am_i":
        return f"I am in {d['place']}, {d['home_distance_m']:.1f} metres from home."
    if tool == "relation":
        return f"The first is {d['a_from_b']}, {d['distance_m']:.1f} metres apart."
    if tool == "path_check":
        if not d["reachable"]:
            return f"I cannot get there; blocked by {d['blocked_by']}."
        return f"It is reachable, {d['path_length_m']:.1f} metres, about {d['eta_s']:.0f} seconds."
    if tool == "status":
        legs = [n.replace("_", " ") for n, s in d["legs"].items() if s != "ok"]
        leg_txt = "; leg problem: " + ", ".join(legs) if legs else ""
        batt = "Battery unknown" if d["battery_pct"] is None else f"Battery {d['battery_pct']:.0f} percent (simulated)"
        return f"{batt}, mode {d['mode']}{leg_txt}."
    if tool == "why":
        return " ".join(e["text"] for e in d["events"]) or "I have nothing logged about that."
    if tool == "recall":
        recs = d["records"]
        if not recs:
            return "I have no record of that."
        return "; ".join(_record(r) for r in recs) + "."
    if tool == "describe_view":
        return d["text"]
    if tool == "report":
        s = d["summary"]
        return f"{s['tool']} {s['state']}: {s['duration_s']:.0f} seconds, {s['distance_m']:.1f} metres, {len(s['objects'])} new objects."
    if tool == "no_go_zone":
        return f"{len(d['zones'])} no-go zones." if d["zones"] else "There are no no-go zones."
    if tool == "save_place":
        return f"Saved this place as {d['place']['name']}."
    if tool == "ask_user":
        return d["question"]
    return ack(tool, args, d)


def summarize_results(entries: list[dict]) -> str:
    """Template reply for all tool results of a turn (grounding fallback)."""
    return " ".join(summarize(e["tool"], e["args"], e["result"]) for e in entries) or "Okay."


def describe_event(ev: dict) -> str | None:
    """Announcement for a backend / executor event, or None if it should stay silent."""
    d, t = ev.get("data", {}), ev["type"]
    if t == "task_finished":
        return _finished(d)
    if t == "task_failed":
        return _finished(d, failed=True)
    if t in ("mission_failed", "mission_finished", "confirmation_needed"):
        return None if t == "mission_finished" else ev["text"]
    if t in ("leg_failure", "path_blocked", "object_missing", "battery_low", "estop"):
        return ev["text"]
    if t == "object_new":
        return ev["text"]
    return None


_DOING = {"go_to": "get there", "return_home": "get home", "walk": "walk", "turn": "turn", "find": "find it", "explore": "finish exploring",
          "patrol": "finish the patrol", "inspect": "inspect it"}


def failure_text(tool: str, reason: str, args: dict | None = None, d: dict | None = None) -> str | None:
    """A plain sentence for a mission step that did not succeed (never "go_to stopped: blocked"). None = nothing to say (the user stopped it
    and has heard "Stopped.", or an emergency stop has its own message)."""
    from .rooms import canon_place
    args = args or {}
    where = ""
    if tool in ("go_to", "inspect", "return_home"):
        t = (d or {}).get("target") or {}
        name = args.get("place") or t.get("place")
        where = f" {canon_place(name)}" if name else (" home" if tool == "return_home" else "")
    if reason in ("cancelled", "estop"):
        return None
    if reason == "blocked":
        if tool in ("go_to", "inspect", "return_home"):
            return f"I couldn't reach{where or ' the goal'}: my path is blocked."
        if tool in ("walk", "turn"):
            return f"I couldn't {tool} as far as you asked: something is in the way."
        return f"I had to stop {_DOING_ING.get(tool, 'that')}: my path is blocked."
    if reason == "no_path":
        return f"I couldn't find a way to{where or ' the goal'}."
    if reason == "timeout":
        return f"I ran out of time trying to {_DOING.get(tool, 'finish that')}."
    if reason in ("stuck", "no_progress"):
        return f"I got stuck and couldn't {_DOING.get(tool, 'finish that')}."
    if reason == "no_go_zone":
        return f"I stopped{(' going to' + where) if where else ''} because the way there leads through a no-go zone."
    if reason == "fall":
        return "I fell over, so I stopped."
    if reason in ("leg_failure", "leg_failed"):
        return "I stopped because a leg has failed."
    return f"I couldn't {_DOING.get(tool, 'finish that')}."


_DOING_ING = {"find": "the search", "explore": "exploring", "patrol": "the patrol", "go_to": "going there", "walk": "walking", "turn": "turning"}
_DONE = {"find": "looked for", "go_to": "gone to", "walk": "walked", "turn": "turned", "patrol": "patrolled", "explore": "explored",
         "return_home": "gone home", "inspect": "inspected"}


def not_done_phrase(tool: str, args: dict) -> str:
    """"looked for the red box" / "gone to room B" for the sentence "I haven't ... yet"."""
    if tool == "find":
        return f"looked for the {obj_name(args['object'])}"
    if tool in ("go_to", "inspect"):
        return f"{_DONE[tool]} {_target_name(args)}"
    return _DONE.get(tool, "done that")


def offer_question(tool: str) -> str:
    return "Shall I search from here?" if tool == "find" else "Shall I try that from here?"


def _finished(d: dict, failed: bool = False) -> str | None:
    tool, reason = d.get("tool", "task"), d.get("stopped_reason", "done")
    if tool in ("go_to", "return_home") and not failed:
        if d.get("close_enough"):
            from .rooms import canon_place
            place = (d.get("target") or {}).get("place")
            if d.get("inside") and place:
                return f"I'm in {canon_place(place)}, close to the goal."
            return f"I'm close to the goal, {d['final_distance_m']:.1f} metres away."
        return f"I have arrived, {d['final_distance_m']:.1f} metres from the goal."
    if tool == "find":
        o = d.get("object")
        if d.get("found") and o:
            return f"Found the {obj_where(o)}."
        if d.get("in_place"):
            return f"I did not find it in {rooms_canon(d['in_place'])} after {d.get('searched_s', 0):.0f} seconds."
        return f"I did not find it after {d.get('searched_s', 0):.0f} seconds."
    if failed:
        return failure_text(tool, reason, d=d)
    return "Done."


def squash(reply: str, max_items: int = 8) -> str:
    """Never print the same item over and over, or a whole memory dump: repeated ';' / '.' segments are listed once and a long list is cut with a count
    (the first incident evaluation printed one sentence ~100 times and 'I recall: ...' with every object memory entry)."""
    if not reply or len(reply) < 160:
        return reply
    head, sep, body = reply.partition(": ")
    if not sep or ";" not in body:
        sents = re.split(r"(?<=[.!?])\s+", reply)
        seen: dict[str, int] = {}
        out = []
        for x in sents:
            k = x.strip().lower()
            if k in seen:
                seen[k] += 1
                continue
            seen[k] = 1
            out.append(x)
        return " ".join(out) if len(out) < len(sents) else reply
    items = [x.strip() for x in body.rstrip(". ").split(";") if x.strip()]
    uniq = list(dict.fromkeys(i.lower() and i for i in items))
    if len(uniq) == len(items) and len(items) <= max_items:
        return reply
    shown = uniq[:max_items]
    more = f"; and {len(uniq) - len(shown)} more" if len(uniq) > len(shown) else ""
    return f"{head}: {'; '.join(shown)}{more}."
