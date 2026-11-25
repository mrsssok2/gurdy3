"""Template answers for the incident tools, built ONLY from the query result (names, rooms, times, counts all come from it). They are the
fallback of the grounding check and the reply when no LLM phrases the answer. `must_include` lists what an LLM-phrased reply has to mention."""
from __future__ import annotations

from typing import Any

MAX_ITEMS = 6


def _cap(s: str) -> str:
    return s[:1].upper() + s[1:]


def _room(r: str | None) -> str:
    return r or "an unnamed spot"


def _when(e: dict) -> str:
    return f"at {e['clock']} ({e['ago_text']})"


def _loc(p: dict | None) -> str:
    if not p:
        return "an unknown spot"
    return p["room"] if p.get("room") else f"({p['x']:.1f}, {p['y']:.1f})"


def _moved_text(e: dict) -> str:
    f, t = e.get("from"), e.get("to")
    if f and t and f.get("room") and t.get("room") and f["room"] != t["room"]:
        where = f"from {f['room']} to {t['room']}"
    elif f and t and t.get("room"):
        where = f"{e['distance_m']:.1f} m within {t['room']}" if e.get("distance_m") is not None else f"within {t['room']}"
    else:
        where = f"from {_loc(f)} to {_loc(t)}"
    return f"the {e['object']} moved {where} {_when(e)}"


def _list(items: list[str], more_word: str = "more") -> str:
    shown = items[:MAX_ITEMS]
    text = "; ".join(shown)
    if len(items) > len(shown):
        text += f"; and {len(items) - len(shown)} {more_word}"
    return text


def phrase(tool: str, d: dict, args: dict | None = None) -> str:
    return _PHRASE[tool](d, args or {})


def _what_changed(d: dict, a: dict) -> str:
    room, since = d.get("room"), d["since_text"]
    scope = f"{since}, in {room}" if room else since
    if d["changes"] == 0:
        if d.get("no_data"):
            if room:
                extra = f" I was last in {room} at {d['last_in_clock']} ({d['last_in_ago_text']})." if d.get("last_in_clock") else ""
                return f"I haven't been in {room} {since}, so I can't tell what changed there.{extra}"
            return f"I haven't recorded anything {since}."
        seen = [r for r in d["rooms_checked"] if not room or r["room"].lower() == room.lower()]
        last = seen[-1]
        return f"Nothing has changed {since}{', in ' + room if room else ''}. I was in {last['room']} at {last['last_clock']} ({last['last_ago_text']})."
    parts = [_moved_text(e) for e in d["moved"]]
    parts += [f"the {e['object']} is missing from {_room(e['room'])}; I looked there {_when(e)} and it was not there" for e in d["missing"]]
    parts += [f"a new {e['object']} appeared in {_room(e['room'])} {_when(e)}" for e in d["new"]]
    parts += [f"the {e['object']} is back in {_room(e['room'])} {_when(e)}" for e in d["returned"]]
    parts += [f"{e['text']} {_when(e)}" for e in d["zones"]]
    n = d["changes"]
    head = f"{_cap(scope)}: {n} change{'s' if n != 1 else ''}. "
    return head + _cap(_list(parts)) + "."


def _last_seen(d: dict, a: dict) -> str:
    if not d["found"]:
        return f"I have no record of a {d['object']}."
    def one(m: dict) -> str:
        loc = f"in {m['room']}" if m.get("room") else f"at ({m['x']:.1f}, {m['y']:.1f})"
        txt = f"the {m['object']} {loc} {_when(m)}"
        return txt + (", but it was not there when I checked later" if m["state"] == "missing" else "")
    ms = d["matches"]
    if len(ms) == 1:
        return f"I last saw {one(ms[0])}."
    return f"I know of {len(ms)} of them. I last saw " + _list([one(m) for m in ms], "more") + "."


def _history(d: dict, a: dict) -> str:
    if not d["found"]:
        return f"I have no record of a {d['object']}."
    obj, still = d["object"], bool(a.get("still"))
    if d["moved"]:
        ms = d["moves"]
        text = _list([_moved_text(m) for m in ms])
        return f"{'No' if still else 'Yes'}. {_cap(text)}."
    seen = d["sightings"]
    missing = d["state"] == "missing"
    state = " It is currently marked missing: it was not there when I checked." if missing else ""
    where = f" I last recorded it in {d['current_room']}." if d.get("current_room") else ""
    times = "once" if seen == 1 else f"{seen} times"
    span = "so far" if d["since_text"] == "ever" else d["since_text"]
    if still and missing:
        return f"No. The {obj} was not there when I checked.{where}"
    if still:
        return f"Yes. I haven't recorded the {obj} moving; I have seen it {times} {span}.{where}"
    return f"I haven't recorded the {obj} moving. I have seen it {times} {span}.{where}{state}"


def _timeline(d: dict, a: dict) -> str:
    if d["no_data"]:
        return f"I haven't recorded anything {d['since_text']}" + (f" in {d['room']}." if d.get("room") else ".")
    lines = " ".join(f"{e['clock']} {e['text'].rstrip('.')}." for e in d["events"])
    head = f"Here is what happened {d['since_text']}" + (f" in {d['room']}" if d.get("room") else "")
    tail = f" (the latest {len(d['events'])} of {d['total']})" if d["truncated"] else ""
    return f"{head}{tail}: {lines}"


def _why(d: dict, a: dict) -> str:
    if not d["found"]:
        return "I have no record of a stop or a failure."
    e = d["event"]
    what = e["text"].rstrip(".")
    if e["type"] == "stop":
        what = "I stopped because you told me to"
    base = f"At {e['clock']} ({e['ago_text']}): {what}."
    if d["has_cause"]:
        c = d["cause"]
        quote = f'You said "{c["text"]}".' if c["type"] == "user_command" else f"Cause: {c['text'].rstrip('.')} (at {c['clock']})."
        return f"{base} {quote}"
    return f"{base} I have no recorded cause for it."


def _unchecked(d: dict, a: dict) -> str:
    if d["no_data"]:
        return "I have no rooms recorded yet."
    def last(r: dict) -> str:
        return f"{r['room']} (last at {r['clock']}, {r['ago_text']})" if not r["never"] else f"{r['room']} (I have no record of being there)"
    if d.get("asked_room"):
        r = (d["unchecked"] + d["recent"])[0]
        return f"I was last in {r['room']} at {r['clock']} ({r['ago_text']})." if not r["never"] else f"I have no record of being in {r['room']}."
    if not d["unchecked"]:
        return f"I have checked every room less than {d['threshold']} ago."
    ok_ = f" Checked recently: {', '.join(r['room'] for r in d['recent'])}." if d["recent"] else ""
    return f"Not checked for {d['threshold']}: " + _list([last(r) for r in d["unchecked"]]) + "." + ok_


_PHRASE = {"what_changed": _what_changed, "last_seen": _last_seen, "object_history": _history, "timeline": _timeline, "incident_why": _why,
           "unchecked_rooms": _unchecked}


def must_include(tool: str, d: dict) -> list[str]:
    """Lower-case fragments an LLM-phrased reply must contain (the facts that carry the answer)."""
    if tool == "what_changed":
        return [e["object"].split()[-1] for k in ("moved", "missing", "new", "returned") for e in d.get(k, [])]
    if tool == "last_seen" and d.get("found"):
        return [d["matches"][0]["clock"]]
    if tool == "object_history" and d.get("found"):
        return ["yes"] if d["moved"] else []
    return []
