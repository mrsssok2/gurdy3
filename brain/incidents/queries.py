"""Query layer over the temporal graph. Pure functions: store + clock in, `Result` out. No prose here (phrasing.py), no LLM.
Every time in a result is given three ways (`wall_t`, `clock` "10:42", `ago_text` "12 minutes ago") so a reply can quote it and the grounding check
can verify it. A question about something the graph has no data for returns `no_data` with the reason, never an empty success."""
from __future__ import annotations

import math
import re
import time
from datetime import datetime
from typing import Any, Callable

from brain.tools.types import Result, fail, ok

from .recorder import pretty
from .store import IncidentStore

CHANGE_TYPES = ("object_moved", "object_missing", "object_seen", "no_go_set", "no_go_cleared")
STORY_TYPES = ("object_moved", "object_missing", "object_seen", "no_go_set", "no_go_cleared", "room_visited", "path_blocked", "stop", "estop",
               "alert", "task_finished", "task_failed")
STOPLIKE = ("estop", "stop", "task_failed")
_CLOCK = re.compile(r"^(?:clock:)?(\d{1,2}):(\d{2})$")
_AGO = re.compile(r"^ago:(\d+(?:\.\d+)?)$")


# ---------------------------------------------------------------- time
def clock_text(wall: float) -> str:
    return datetime.fromtimestamp(wall).strftime("%H:%M")


def ago_text(delta_s: float) -> str:
    s = max(0.0, delta_s)
    if s < 60:
        return "just now"
    m = round(s / 60)
    if m < 60:
        return f"{m} minute{'s' if m != 1 else ''} ago"
    h = round(s / 3600)
    if h < 24:
        return f"{h} hour{'s' if h != 1 else ''} ago"
    d = round(s / 86400)
    return f"{d} day{'s' if d != 1 else ''} ago"


def window_text(h: float) -> str:
    """'an hour' / '30 minutes' for a duration, used in 'not checked for ...'."""
    if h < 90:
        return "a minute" if round(h) <= 1 else f"{round(h)} seconds"
    m = round(h / 60)
    if m < 60:
        return f"{m} minutes"
    hh = round(h / 3600)
    return "an hour" if hh == 1 else f"{hh} hours"


def resolve_since(spec: str | None, store: IncidentStore, now: float, default: str = "today") -> tuple[float | None, str] | None:
    """Window start (epoch seconds, None = everything) and its words for a reply, or None when `spec` is not understood.
    today | session | away | all | clock:HH:MM (or HH:MM) | ago:<seconds>"""
    spec = (spec or default).strip().lower()
    if spec == "all":
        return None, "ever"
    if spec == "today":
        d = datetime.fromtimestamp(now).replace(hour=0, minute=0, second=0, microsecond=0)
        return d.timestamp(), "today"
    if spec == "session":
        return store.started_wall, "since you started me"
    if spec == "away":
        # anchored to the last command that STARTED a mission (the user left while it ran), else the session start; questions never move it
        cmds = [c for c in store.events(etypes=["user_command"]) if c["data"].get("starts_task")]
        return (cmds[-1]["wall_t"] if cmds else store.started_wall), "while you were away"
    if m := _CLOCK.match(spec):
        h, mi = int(m.group(1)), int(m.group(2))
        if h > 23 or mi > 59:
            return None
        t = datetime.fromtimestamp(now).replace(hour=h, minute=mi, second=0, microsecond=0).timestamp()
        if t > now:
            t -= 86400
        return t, f"since {h:02d}:{mi:02d}"
    if m := _AGO.match(spec):
        secs = float(m.group(1))
        return now - secs, f"in the last {window_text(secs)}"
    return None


def _bad_since(spec: Any) -> Result:
    return fail("INVALID_ARGS", f"I do not understand the time window {spec!r}.", problems=[f"since {spec!r}"])


def fmt_event(e: dict, now: float) -> dict:
    d = e["data"]
    out = {"id": e["id"], "type": e["etype"], "text": e["label"], "severity": e["severity"], "wall_t": e["wall_t"], "sim_t": e["sim_t"],
           "clock": clock_text(e["wall_t"]), "ago_text": ago_text(now - e["wall_t"]), "ago_s": round(now - e["wall_t"]), "room": e["room"]}
    if e["cls"]:
        out["object"] = pretty(e["cls"], e["color"])
    if e["etype"] == "object_moved":
        out["from"], out["to"], out["distance_m"] = d.get("from"), d.get("to"), d.get("distance_m")
    elif e["x"] is not None:
        out["x"], out["y"] = round(e["x"], 2), round(e["y"], 2)
    for k in ("reason", "tool", "kind", "target", "first", "returned", "why", "stayed_s", "text"):
        if k in d and d[k] is not None:
            out[k] = d[k]
    return out


def _object_ids(store: IncidentStore, ref: dict) -> list[int]:
    """Node ids of the objects matching a {class, color?} reference, merged duplicates included."""
    cls, color = ref.get("class"), (ref.get("color") or "").lower()
    ids = []
    for n in store.nodes("object"):
        if n["state"] == "merged" or n["cls"] != cls or (color and (n["color"] or "").lower() != color):
            continue
        ids.append(n["id"])
    return ids


def _group(store: IncidentStore, nid: int) -> list[int]:
    return [nid] + [n["id"] for n in store.nodes("object", state="merged") if n["data"].get("merged_into") == nid]


# ---------------------------------------------------------------- what_changed
def _visited_rooms(store: IncidentStore, since: float | None, now: float) -> dict[str, dict]:
    out: dict[str, dict] = {}
    for e in store.events(since_wall=since, etypes=["room_visited", "room_left"]):
        out[e["room"]] = {"room": e["room"], "last_clock": clock_text(e["wall_t"]), "last_ago_text": ago_text(now - e["wall_t"]),
                                "last_ago_s": round(now - e["wall_t"]), "wall_t": e["wall_t"]}
    return out


def what_changed(store: IncidentStore, now: float, room: str | None = None, since: str | None = None) -> Result:
    win = resolve_since(since, store, now)
    if win is None:
        return _bad_since(since)
    since_wall, since_words = win
    evs = [e for e in store.events(since_wall=since_wall, etypes=CHANGE_TYPES) if not e["data"].get("superseded")]

    def in_room(e: dict) -> bool:
        if not room:
            return True
        rl = room.lower()
        if e["etype"] == "object_moved":
            return rl in ((e["data"].get("from") or {}).get("room") or "").lower() or rl in ((e["data"].get("to") or {}).get("room") or "").lower()
        return (e["room"] or "").lower() == rl

    evs = [e for e in evs if in_room(e)]
    moved: dict[int, dict] = {}
    gone: dict[int, dict] = {}
    new: list[dict] = []
    back: list[dict] = []
    config: list[dict] = []
    for e in evs:
        oid = next((g["dst"] for g in store.edges(src=e["id"], rel="about")), None)
        if e["etype"] == "object_moved":
            cur = moved.get(oid)
            f = fmt_event(e, now)
            if cur:
                f["from"] = cur["from"]                       # first position of the window -> last
                if f["from"] and f["to"]:
                    f["distance_m"] = round(math.hypot(f["to"]["x"] - f["from"]["x"], f["to"]["y"] - f["from"]["y"]), 2)
            moved[oid] = f
            gone.pop(oid, None)
        elif e["etype"] == "object_missing":
            gone[oid] = fmt_event(e, now)
        elif e["etype"] == "object_seen":
            f = fmt_event(e, now)
            if e["data"].get("first"):
                new.append(f)
            elif e["data"].get("returned"):
                back.append(f)
                gone.pop(oid, None)
        else:
            config.append(fmt_event(e, now))
    moved_names = {m["object"] for m in moved.values()}
    new = [n for n in new if n["object"] not in moved_names]                # first seen AND moved inside the window: the move is the news
    visited = _visited_rooms(store, since_wall, now)
    changes = len(moved) + len(gone) + len(new) + len(back) + len(config)
    data: dict[str, Any] = {"since_text": since_words, "room": room, "moved": list(moved.values()), "missing": list(gone.values()), "new": new,
                            "returned": back, "zones": config, "changes": changes, "rooms_checked": [v for v in visited.values()]}
    if changes == 0:
        checked_room = (room.lower() in {r.lower() for r in visited}) if room else bool(visited)
        data["no_data"] = not checked_room
        data["reason"] = None if checked_room else ("not_visited" if room else "no_activity")
        if room and not checked_room:
            last = store.nodes("room", key=room.lower())
            if last and last[0]["wall_t"]:
                data["last_in_clock"], data["last_in_ago_text"] = clock_text(last[0]["wall_t"]), ago_text(now - last[0]["wall_t"])
                data["last_in_ago_s"] = round(now - last[0]["wall_t"])
    return ok(**data)


# ---------------------------------------------------------------- last_seen / history
def last_seen(store: IncidentStore, now: float, object: dict) -> Result:       # noqa: A002  (the tool argument is called `object`)
    out = []
    for nid in _object_ids(store, object):
        n = store.node(nid)
        ev = [e for e in store.events(object_id=nid, etypes=["object_seen", "object_moved"])]
        for g in _group(store, nid)[1:]:
            ev += store.events(object_id=g, etypes=["object_seen"])
        t = max([e["wall_t"] for e in ev] + [n["wall_t"] or 0])
        out.append({"object": pretty(n["cls"], n["color"]), "class": n["cls"], "color": n["color"], "room": n["room"], "x": round(n["x"], 2), "y": round(n["y"], 2),
                    "state": n["state"], "wall_t": t, "clock": clock_text(t), "ago_text": ago_text(now - t), "ago_s": round(now - t), "object_id": nid})
    if not out:
        return ok(found=False, object=pretty(object.get("class"), object.get("color")), matches=[])
    out.sort(key=lambda r: -r["wall_t"])
    return ok(found=True, object=out[0]["object"], matches=out)


def object_history(store: IncidentStore, now: float, object: dict, since: str | None = None) -> Result:       # noqa: A002
    win = resolve_since(since, store, now, default="all")
    if win is None:
        return _bad_since(since)
    ids = _object_ids(store, object)
    if not ids:
        return ok(found=False, object=pretty(object.get("class"), object.get("color")), events=[], moved=False)
    evs: list[dict] = []
    for nid in ids:
        for g in _group(store, nid):
            evs += store.events(since_wall=win[0], object_id=g, etypes=["object_seen", "object_moved", "object_missing"])
    evs = [e for e in sorted(evs, key=lambda e: (e["wall_t"], e["id"])) if not e["data"].get("superseded")]
    moves = [fmt_event(e, now) for e in evs if e["etype"] == "object_moved"]
    n = store.node(ids[0])
    return ok(found=True, object=pretty(n["cls"], n["color"]), since_text=win[1], events=[fmt_event(e, now) for e in evs][-12:], moved=bool(moves), moves=moves,
              sightings=sum(1 for e in evs if e["etype"] == "object_seen"), current_room=n["room"], state=n["state"],
              first_seen_clock=clock_text(evs[0]["wall_t"]) if evs else None, copies=len(ids))


# ---------------------------------------------------------------- timeline
def timeline(store: IncidentStore, now: float, since: str | None = None, room: str | None = None, limit: int = 10, object: dict | None = None) -> Result:   # noqa: A002
    win = resolve_since(since, store, now)
    if win is None:
        return _bad_since(since)
    oids = {g for nid in _object_ids(store, object) for g in _group(store, nid)} if object else None
    evs = []
    for e in store.events(since_wall=win[0], etypes=STORY_TYPES):
        d = e["data"]
        if e["etype"] == "object_seen" and not (d.get("first") or d.get("returned")) or d.get("superseded"):
            continue
        if room and not (room.lower() in ((e["room"] or "").lower(), ((d.get("from") or {}).get("room") or "").lower(), ((d.get("to") or {}).get("room") or "").lower())):
            continue
        if oids is not None and not any(g["dst"] in oids for g in store.edges(src=e["id"], rel="about")):
            continue
        evs.append(e)
    total = len(evs)
    shown = [fmt_event(e, now) for e in evs[-limit:]]
    return ok(since_text=win[1], room=room, events=shown, total=total, truncated=total > len(shown), no_data=total == 0)


# ---------------------------------------------------------------- why
def _chain(store: IncidentStore, eid: int, depth: int = 4) -> list[int]:
    out, cur = [eid], eid
    for _ in range(depth):
        nxt = next((e["dst"] for e in store.edges(src=cur, rel="caused_by")), None)
        if nxt is None or nxt in out:
            break
        out.append(nxt)
        cur = nxt
    return out


def why(store: IncidentStore, now: float, about: str = "stop", at: str | None = None, event_id: int | None = None) -> Result:
    if event_id is not None:
        target = store.node(event_id)
        if not target or target["kind"] != "event":
            return ok(found=False, about=about, chain=[])
    else:
        types = {"stop": ("estop", "stop", "task_failed"), "failure": ("task_failed",), "alert": ("alert",), "last_event": STORY_TYPES}.get(about, STOPLIKE)
        cands = store.events(etypes=types)
        if about == "stop":
            # "why did you stop": the stop the user gave (or the e-stop) beats a task failure / timeout; those only count when there was no such stop
            cands = store.events(etypes=("estop", "stop")) or cands
        if at:
            m = _CLOCK.match(at)
            if not m:
                return _bad_since(at)
            ref = datetime.fromtimestamp(now).replace(hour=int(m.group(1)), minute=int(m.group(2)), second=0).timestamp()
            cands = [c for c in cands if abs(c["wall_t"] - ref) <= 1800] if cands else cands
            target = min(cands, key=lambda c: abs(c["wall_t"] - ref)) if cands else None
        else:
            target = cands[-1] if cands else None
        if target is None:
            return ok(found=False, about=about, chain=[])
    chain = [store.node(i) for i in _chain(store, target["id"])]
    return ok(found=True, about=about, event=fmt_event(target, now), chain=[fmt_event(e, now) for e in chain], has_cause=len(chain) > 1,
              cause=fmt_event(chain[1], now) if len(chain) > 1 else None)


# ---------------------------------------------------------------- unchecked rooms
def unchecked_rooms(store: IncidentStore, now: float, known_rooms: list[str], older_than_s: float = 3600.0, room: str | None = None) -> Result:
    rooms = sorted({r for r in known_rooms if r} | {n["label"] for n in store.nodes("room")}, key=str.lower)
    if room:
        rooms = [r for r in rooms if r.lower() == room.lower()] or [room]
    if not rooms:
        return ok(no_data=True, rooms=[], threshold=window_text(older_than_s))
    by_name = {n["label"].lower(): n for n in store.nodes("room")}
    old, recent = [], []
    for r in rooms:
        n = by_name.get(r.lower())
        t = n["wall_t"] if n and n["wall_t"] else None
        rec = {"room": r, "never": t is None, "wall_t": t, "clock": clock_text(t) if t else None, "ago_text": ago_text(now - t) if t else None,
               "ago_s": round(now - t) if t else None}
        (recent if t is not None and now - t < older_than_s else old).append(rec)
    return ok(no_data=False, threshold=window_text(older_than_s), threshold_s=older_than_s, unchecked=old, recent=recent, since_started_clock=clock_text(store.started_wall),
              asked_room=room)
