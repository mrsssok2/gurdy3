"""Incident Graph (temporal memory): what happened, when, where. See docs/phase6_incident_graph.md.

store.py     thread-safe SQLite temporal graph (nodes: object / room / robot / user / event; edges: seen_in, moved_from, moved_to, caused_by, before, ...)
recorder.py  turns backend events, object sightings, poses and user commands into graph events (move / missing detection)
queries.py   what_changed, last_seen, object_history, timeline, why, unchecked_rooms: deterministic, return plain dicts
phrasing.py  template answers built only from query results
nlu.py       deterministic recognition of incident questions (no LLM)
backend.py   IncidentBackend: wraps any RobotBackend and records while it runs
"""
