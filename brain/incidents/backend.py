"""IncidentBackend: wraps any RobotBackend (mock, ROS, MemoryBackend over either) and records while it runs. Everything else is delegated untouched.

  poll_events()        records the backend events, then samples the pose and the object records (throttled to one sample per TICK_S of sim time)
  no_go_zone(...)      a successful add / remove becomes no_go_set / no_go_cleared
  note_user_command()  called by the agent AFTER it answered (so "while I was away" means since the PREVIOUS command)
  incident_query()     what_changed / last_seen / object_history / timeline / incident_why / unchecked_rooms for the tool layer
`MEMORY=false` = a store on ":memory:" (records within the session, nothing survives a restart)."""
from __future__ import annotations

import threading
from typing import Any

from brain.tools.types import Result, fail, ok

from . import queries
from .recorder import TICK_S, IncidentRecorder
from .store import IncidentStore
from .tools import INCIDENT_TOOLS


class IncidentBackend:
    def __init__(self, inner: Any, store: IncidentStore, recorder: IncidentRecorder | None = None):
        self._inner, self.store = inner, store
        self.recorder = recorder or IncidentRecorder(store, is_room=self._is_room)
        self._lock = threading.RLock()
        self._last_tick = -1e9

    def __getattr__(self, name: str):
        return getattr(self._inner, name)

    def _is_room(self, name: str) -> bool:
        rooms = getattr(getattr(self._inner, "world", None), "rooms", None)
        if rooms is not None and name in rooms:
            return True
        return bool(name) and name.lower().startswith("room")

    def known_rooms(self) -> list[str]:
        return [p for p in self._inner.known_places() if self._is_room(p)]

    # ------------------------------------------------------------------ recording
    def poll_events(self) -> list[dict]:
        with self._lock:
            evs = self._inner.poll_events()
            now = self._inner.now()
            self.recorder.backend_events(evs, now)
            self.tick(force=bool(evs))
            return evs

    def tick(self, force: bool = False) -> None:
        """Sample pose + object records now (the agent's pump loop calls poll_events often; tests and scenarios call this directly)."""
        with self._lock:
            now = self._inner.now()
            if not force and now - self._last_tick < TICK_S:
                return
            self._last_tick = now
            pose = self._inner.where_am_i()
            if pose.ok:
                self.recorder.observe_pose(pose.data, now)
            res = self._inner.recall("object")
            if res.ok:
                self.recorder.observe_objects(res.data["records"], now)

    def no_go_zone(self, action: str, place: str | None = None, around_object_id: int | None = None, radius_m: float = 0.8) -> Result:
        res = self._inner.no_go_zone(action, place=place, around_object_id=around_object_id, radius_m=radius_m)
        if res.ok and action in ("add", "remove"):
            target = place or (f"around object {around_object_id}" if around_object_id else "an area")
            self.recorder.no_go(action, target, self._inner.now())
        return res

    def note_user_command(self, text: str) -> None:
        with self._lock:
            self.recorder.user_command(text, self._inner.now())

    # ------------------------------------------------------------------ queries
    def incident_query(self, name: str, args: dict) -> Result:
        if name not in INCIDENT_TOOLS:
            return fail("UNKNOWN_TOOL", f"no incident tool named {name}")
        with self._lock:
            self.tick(force=True)                                   # what the user asks about includes what happened a moment ago
            now = self.store.wall()
            a = dict(args)
            if name == "what_changed":
                return queries.what_changed(self.store, now, a.get("room"), a.get("since"))
            if name == "last_seen":
                return queries.last_seen(self.store, now, a["object"])
            if name == "object_history":
                return queries.object_history(self.store, now, a["object"], a.get("since"))
            if name == "timeline":
                return queries.timeline(self.store, now, a.get("since"), a.get("room"), a.get("limit", 10), a.get("object"))
            if name == "incident_why":
                return queries.why(self.store, now, a["about"], a.get("at"), a.get("event_id"))
            return queries.unchecked_rooms(self.store, now, self.known_rooms(), a.get("older_than_s", 3600.0), a.get("room"))

    def moved_objects(self, since_wall: float | None = None) -> list[dict]:
        return self.recorder.moved_objects(since_wall)


def with_incidents(inner: Any, db_path: str | None, map_id: str, wall=None) -> IncidentBackend:
    """Wrap `inner` with the incident graph. `db_path=None` (MEMORY=false) keeps the graph in RAM only: it starts empty and dies with the process."""
    import time
    store = IncidentStore(db_path or ":memory:", map_id, wall or time.time)
    return IncidentBackend(inner, store)
