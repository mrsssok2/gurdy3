"""Timeline panel / "what changed" button: the same query tools the agent uses (brain.incidents), answered from the temporal graph.
Shared by the HTTP server (GET /api/timeline, /api/changes) and the in-browser demo. Query args (all optional): since (today | session | away | all |
clock:HH:MM | ago:SECONDS), room, class + color (one object), limit."""
from __future__ import annotations

from typing import Any


def answer(backend: Any, which: str, query: dict[str, str], lock=None) -> dict:
    """`which` = "timeline" | "changes". `query`: plain {name: value} strings. `available: false` = this robot keeps no timeline."""
    if not hasattr(backend, "incident_query"):
        return {"available": False}
    args: dict[str, Any] = {k: query[k] for k in ("since", "room") if query.get(k)}
    if which == "timeline":
        if query.get("class"):
            args["object"] = {"class": query["class"], **({"color": query["color"]} if query.get("color") else {})}
        try:
            args["limit"] = max(1, min(50, int(query.get("limit") or 30)))
        except ValueError:
            args["limit"] = 30
    if lock is not None and not lock.acquire(timeout=1.0):                      # the mock backend is not thread-safe: same lock as the telemetry
        return {"available": True, "busy": True}
    try:
        res = backend.incident_query("timeline" if which == "timeline" else "what_changed", args)
    finally:
        if lock is not None:
            lock.release()
    if not res.ok:
        return {"available": True, "error": res.error["message"]}
    out: dict[str, Any] = {"available": True, "data": res.data}
    if which == "changes":
        from brain.incidents.phrasing import phrase
        out["text"] = phrase("what_changed", res.data)
    return out
