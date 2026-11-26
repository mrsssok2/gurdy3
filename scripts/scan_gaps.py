#!/usr/bin/env python3
"""Measure /scan inter-arrival times and recommend the safety-shield scan_timeout (stdlib only; run tomorrow).

    python3 scripts/scan_gaps.py BAG_DIR_OR.db3 [--topic /scan]      # rosbag2 sqlite3 (.db3): RECORD (receive) time
    python3 scripts/scan_gaps.py times.log [--column 0]              # text: one time (s) per line, e.g.
        ros2 topic echo /scan --field header.stamp ... or `ros2 topic hz`-style logs converted to times
Receive time is what the shield uses (time.monotonic at arrival), so bag record time is the right clock. MCAP bags are
not supported here (use `ros2 bag convert` to sqlite3, or export times to a text file).
Recording plan: 10 min with the detector + Nav2 running (the load that caused the stops), `ros2 bag record /scan`.
"""
import argparse
import sqlite3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gurdy_core"))
from gurdy_core.scan_timing import choose_timeout, gaps, stats  # noqa: E402


def times_from_db3(path: Path, topic: str) -> list:
    db = path if path.suffix == ".db3" else next(path.glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    row = con.execute("SELECT id FROM topics WHERE name = ?", (topic,)).fetchone()
    if row is None:
        raise SystemExit(f"topic {topic} not in {db}: {[r[0] for r in con.execute('SELECT name FROM topics')]}")
    return [t / 1e9 for (t,) in con.execute("SELECT timestamp FROM messages WHERE topic_id = ? ORDER BY timestamp", row)]


def times_from_text(path: Path, column: int) -> list:
    out = []
    for ln in path.read_text().splitlines():
        parts = ln.replace(",", " ").split()
        try:
            out.append(float(parts[column]))
        except (IndexError, ValueError):
            continue
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("source")
    ap.add_argument("--topic", default="/scan")
    ap.add_argument("--column", type=int, default=0)
    a = ap.parse_args(argv)
    p = Path(a.source)
    t = times_from_db3(p, a.topic) if (p.is_dir() or p.suffix == ".db3") else times_from_text(p, a.column)
    g = gaps(t)
    s = stats(g)
    d = choose_timeout(g)
    print(f"{len(t)} messages, {s.get('n', 0)} gaps")
    if s.get("n"):
        print("gap s: mean {mean:.3f}  p50 {p50:.3f}  p95 {p95:.3f}  p99 {p99:.3f}  max {max:.3f}".format(**s))
        print(f"gaps > 0.3 s: {s['n_over_0.3']}, > 0.5 s: {s['n_over_0.5']}, > 1.0 s: {s['n_over_1.0']}")
    print(f"recommended scan_timeout: {d['timeout']} s  ({d['reason']})")
    return d


if __name__ == "__main__":
    main()
