"""SQLite temporal graph. One table of nodes, one of edges; every row carries `map_id` (coordinates of one map mean nothing in another).

Nodes   kind = object | room | robot | user | event.  Event nodes carry `etype` (object_seen, object_moved, object_missing, room_visited, ...),
        sim time `sim_t` (the robot's clock) AND wall time `wall_t` (epoch seconds; survives restarts), room, position (x, y) and for moves the
        destination (x2, y2), severity, text and a JSON `data` blob. Object nodes: class, colour, last position / room / state.
Edges   seen_in (object -> room, at the time of the sighting), about (event -> object), in (event -> room), moved_from / moved_to (move event -> room),
        caused_by (event -> event), before (earlier event -> next event), issued_by (user command -> user), by (event -> robot).
        `after` is the reverse of `before` and is answered by `edges(dst=...)` rather than stored twice.
Thread-safe like brain.memory.MemoryStore: one connection, an RLock around every public method, plain dicts out."""
from __future__ import annotations

import functools
import json
import sqlite3
import threading
import time
from pathlib import Path
from typing import Any, Callable, Iterable

SCHEMA = """
CREATE TABLE IF NOT EXISTS nodes (id INTEGER PRIMARY KEY, map_id TEXT NOT NULL, kind TEXT NOT NULL, key TEXT, label TEXT, etype TEXT,
  cls TEXT, color TEXT, room TEXT, x REAL, y REAL, x2 REAL, y2 REAL, sim_t REAL, wall_t REAL, session TEXT, severity TEXT, state TEXT, data TEXT);
CREATE INDEX IF NOT EXISTS nodes_kind ON nodes (map_id, kind, wall_t);
CREATE INDEX IF NOT EXISTS nodes_key ON nodes (map_id, kind, key);
CREATE TABLE IF NOT EXISTS edges (id INTEGER PRIMARY KEY, map_id TEXT NOT NULL, src INTEGER NOT NULL, dst INTEGER NOT NULL, rel TEXT NOT NULL,
  sim_t REAL, wall_t REAL, data TEXT);
CREATE INDEX IF NOT EXISTS edges_src ON edges (src, rel);
CREATE INDEX IF NOT EXISTS edges_dst ON edges (dst, rel);
"""
NODE_COLS = ("kind", "key", "label", "etype", "cls", "color", "room", "x", "y", "x2", "y2", "sim_t", "wall_t", "session", "severity", "state", "data")
REL = ("seen_in", "about", "in", "moved_from", "moved_to", "caused_by", "before", "issued_by", "by")


def _locked(fn):
    @functools.wraps(fn)
    def wrapper(self, *a, **kw):
        with self._lock:
            return fn(self, *a, **kw)
    return wrapper


def _row(r: sqlite3.Row) -> dict:
    d = dict(r)
    d["data"] = json.loads(d["data"]) if d.get("data") else {}
    return d


class IncidentStore:
    def __init__(self, path: str | Path, map_id: str, wall: Callable[[], float] = time.time):
        """`path=":memory:"` = nothing survives a restart (MEMORY=false). `wall` is injectable so tests control time."""
        self.path, self.map_id, self.wall = str(path), map_id, wall
        if self.path != ":memory:":
            Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self.db = sqlite3.connect(self.path, check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.executescript(SCHEMA)
        self.session = f"s{int(self.wall() * 1000)}"
        self.started_wall = self.wall()
        self.robot_id = self.ensure_node("robot", "gurdy", "Gurdy")
        self.user_id = self.ensure_node("user", "user", "the user")

    @_locked
    def close(self) -> None:
        self.db.commit()
        self.db.close()

    # ------------------------------------------------------------------ nodes
    @_locked
    def ensure_node(self, kind: str, key: str, label: str | None = None, **fields: Any) -> int:
        r = self.db.execute("SELECT id FROM nodes WHERE map_id=? AND kind=? AND key=?", (self.map_id, kind, key)).fetchone()
        if r:
            return r["id"]
        return self.add_node(kind, key=key, label=label or key, **fields)

    @_locked
    def add_node(self, kind: str, **fields: Any) -> int:
        fields = {k: (json.dumps(v, default=str) if k == "data" and not isinstance(v, str) else v) for k, v in fields.items()}
        unknown = set(fields) - set(NODE_COLS)
        if unknown:
            raise ValueError(f"unknown node fields {sorted(unknown)}")
        fields.setdefault("session", self.session)
        cols = ["map_id", "kind", *[k for k in fields if k != "kind"]]
        vals = [self.map_id, kind, *[v for k, v in fields.items() if k != "kind"]]
        cur = self.db.execute(f"INSERT INTO nodes ({','.join(cols)}) VALUES ({','.join('?' * len(cols))})", vals)
        self.db.commit()
        return cur.lastrowid

    @_locked
    def update_node(self, node_id: int, **fields: Any) -> None:
        if not fields:
            return
        fields = {k: (json.dumps(v, default=str) if k == "data" and not isinstance(v, str) else v) for k, v in fields.items()}
        self.db.execute(f"UPDATE nodes SET {','.join(k + '=?' for k in fields)} WHERE id=?", [*fields.values(), node_id])
        self.db.commit()

    @_locked
    def node(self, node_id: int) -> dict | None:
        r = self.db.execute("SELECT * FROM nodes WHERE id=?", (node_id,)).fetchone()
        return _row(r) if r else None

    @_locked
    def nodes(self, kind: str, **where: Any) -> list[dict]:
        sql, args = "SELECT * FROM nodes WHERE map_id=? AND kind=?", [self.map_id, kind]
        for k, v in where.items():
            sql += f" AND {k}=?"
            args.append(v)
        return [_row(r) for r in self.db.execute(sql + " ORDER BY id", args)]

    @_locked
    def ensure_room(self, name: str) -> int:
        return self.ensure_node("room", name.lower(), name)

    # ------------------------------------------------------------------ events
    @_locked
    def add_event(self, etype: str, text: str, sim_t: float, wall_t: float | None = None, severity: str = "info", room: str | None = None,
                  x: float | None = None, y: float | None = None, x2: float | None = None, y2: float | None = None,
                  object_id: int | None = None, cls: str | None = None, color: str | None = None, data: dict | None = None) -> int:
        """Insert an event node and its structural edges (about, in, before the previous event, by robot)."""
        wall_t = self.wall() if wall_t is None else wall_t
        prev = self.db.execute("SELECT id FROM nodes WHERE map_id=? AND kind='event' ORDER BY id DESC LIMIT 1", (self.map_id,)).fetchone()
        eid = self.add_node("event", key=etype, label=text, etype=etype, sim_t=sim_t, wall_t=wall_t, severity=severity, room=room, x=x, y=y,
                            x2=x2, y2=y2, cls=cls, color=color, data=data or {})
        if prev:
            self.add_edge(prev["id"], eid, "before", sim_t, wall_t)
        if object_id:
            self.add_edge(eid, object_id, "about", sim_t, wall_t)
        if room:
            self.add_edge(eid, self.ensure_room(room), "in", sim_t, wall_t)
        self.add_edge(eid, self.robot_id, "by", sim_t, wall_t)
        return eid

    @_locked
    def events(self, since_wall: float | None = None, until_wall: float | None = None, etypes: Iterable[str] | None = None,
               room: str | None = None, object_id: int | None = None, session: str | None = None) -> list[dict]:
        sql, args = "SELECT * FROM nodes WHERE map_id=? AND kind='event'", [self.map_id]
        if since_wall is not None:
            sql += " AND wall_t>=?"; args.append(since_wall)
        if until_wall is not None:
            sql += " AND wall_t<=?"; args.append(until_wall)
        if etypes:
            et = list(etypes)
            sql += f" AND etype IN ({','.join('?' * len(et))})"; args.extend(et)
        if room:
            sql += " AND LOWER(room)=?"; args.append(room.lower())
        if session:
            sql += " AND session=?"; args.append(session)
        if object_id:
            sql += " AND id IN (SELECT src FROM edges WHERE dst=? AND rel='about')"; args.append(object_id)
        return [_row(r) for r in self.db.execute(sql + " ORDER BY wall_t, id", args)]

    # ------------------------------------------------------------------ edges
    @_locked
    def add_edge(self, src: int, dst: int, rel: str, sim_t: float | None = None, wall_t: float | None = None, data: dict | None = None) -> int:
        if rel not in REL:
            raise ValueError(f"unknown relation {rel!r}")
        cur = self.db.execute("INSERT INTO edges (map_id, src, dst, rel, sim_t, wall_t, data) VALUES (?,?,?,?,?,?,?)",
                              (self.map_id, src, dst, rel, sim_t, wall_t, json.dumps(data) if data else None))
        self.db.commit()
        return cur.lastrowid

    @_locked
    def edges(self, src: int | None = None, dst: int | None = None, rel: str | None = None) -> list[dict]:
        sql, args = "SELECT * FROM edges WHERE map_id=?", [self.map_id]
        for col, v in (("src", src), ("dst", dst), ("rel", rel)):
            if v is not None:
                sql += f" AND {col}=?"; args.append(v)
        return [_row(r) for r in self.db.execute(sql + " ORDER BY id", args)]

    @_locked
    def count(self) -> dict[str, int]:
        n = self.db.execute("SELECT COUNT(*) c FROM nodes WHERE map_id=? AND kind='event'", (self.map_id,)).fetchone()["c"]
        e = self.db.execute("SELECT COUNT(*) c FROM edges WHERE map_id=?", (self.map_id,)).fetchone()["c"]
        return {"events": n, "edges": e}

    def to_networkx(self):
        """Optional: the whole graph as a networkx MultiDiGraph (networkx is not a dependency; imported on demand)."""
        import networkx as nx
        g = nx.MultiDiGraph()
        with self._lock:
            for r in self.db.execute("SELECT * FROM nodes WHERE map_id=?", (self.map_id,)):
                g.add_node(r["id"], **{k: r[k] for k in r.keys() if k != "id"})
            for r in self.db.execute("SELECT * FROM edges WHERE map_id=?", (self.map_id,)):
                g.add_edge(r["src"], r["dst"], rel=r["rel"], sim_t=r["sim_t"], wall_t=r["wall_t"])
        return g
