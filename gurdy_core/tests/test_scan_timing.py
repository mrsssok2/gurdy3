"""Shield stale-scan timeout: decision rule, gap tool (synthetic rosbag2 sqlite + text log), patch applies to main."""
import random
import sqlite3
import subprocess
import sys
from pathlib import Path

import pytest

from gurdy_core.scan_timing import CAP_S, FLOOR_S, choose_timeout, gaps, percentile, stats, validate_timeout

ROOT = Path(__file__).resolve().parents[2]


def scan_times(n=3000, rate=10.0, late_every=300, late=0.6, seed=0):
    rng = random.Random(seed)
    t, out = 0.0, []
    for i in range(n):
        t += 1 / rate + rng.gauss(0, 0.004) + (late if late_every and i % late_every == 0 else 0)
        out.append(t)
    return out


def test_rule_normal_load_keeps_floor():
    d = choose_timeout(gaps(scan_times(late_every=0)))
    assert d["timeout"] == FLOOR_S and not d["capped"]


def test_rule_detector_load_raises_to_p99_times_margin():
    # 2 % of the gaps 0.55-0.65 s (the live test: stops < 0.3 s beyond the 0.5 s timeout)
    rng = random.Random(1)
    g = [0.1 + rng.gauss(0, 0.004) for _ in range(2000)] + [rng.uniform(0.55, 0.65) for _ in range(40)]
    d = choose_timeout(g)
    assert 0.65 < d["timeout"] <= 0.8 and not d["capped"]
    assert d["timeout"] == pytest.approx(1.2 * percentile(g, 99), abs=1e-3)


def test_rule_caps_at_one_second_and_says_fix_the_cause():
    g = [0.1] * 900 + [1.5] * 100
    d = choose_timeout(g)
    assert d["timeout"] == CAP_S and d["capped"] and "FIX" in d["reason"]


def test_rule_too_few_samples():
    assert choose_timeout([0.1] * 20)["timeout"] == FLOOR_S


def test_validate_timeout():
    assert validate_timeout(0.5) == (0.5, None)
    assert validate_timeout(2.0)[0] == CAP_S and validate_timeout(2.0)[1]
    assert validate_timeout(0.0)[0] == FLOOR_S and validate_timeout(float("nan"))[0] == FLOOR_S


def test_stats_counts():
    s = stats([0.1, 0.2, 0.4, 0.6, 1.2])
    assert s["n_over_0.3"] == 3 and s["n_over_0.5"] == 2 and s["n_over_1.0"] == 1 and s["max"] == 1.2


def _tool():
    sys.path.insert(0, str(ROOT / "scripts"))
    import scan_gaps
    return scan_gaps


def test_tool_reads_rosbag2_sqlite(tmp_path):
    db = tmp_path / "bag_0.db3"
    con = sqlite3.connect(db)
    con.execute("CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT, type TEXT, serialization_format TEXT, offered_qos_profiles TEXT)")
    con.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)")
    con.execute("INSERT INTO topics VALUES (1, '/scan', 'sensor_msgs/msg/LaserScan', 'cdr', '')")
    con.execute("INSERT INTO topics VALUES (2, '/odom', 'nav_msgs/msg/Odometry', 'cdr', '')")
    for i, t in enumerate(scan_times(500, late_every=100)):
        con.execute("INSERT INTO messages (topic_id, timestamp, data) VALUES (1, ?, x'00')", (int(t * 1e9),))
        con.execute("INSERT INTO messages (topic_id, timestamp, data) VALUES (2, ?, x'00')", (int(t * 1e9) + 7,))
    con.commit()
    con.close()
    d = _tool().main([str(tmp_path)])
    assert d["timeout"] >= FLOOR_S and d["p"] is not None


def test_tool_reads_text_log(tmp_path):
    f = tmp_path / "times.log"
    f.write_text("\n".join(f"{t:.6f}" for t in scan_times(400)) + "\n# comment\n")
    assert _tool().main([str(f)])["p"] is not None


def test_patch_applies_to_main(tmp_path):
    """The node patch (docs/results/data/phase9f/shield_scan_timeout.patch) applies cleanly to main's shield node."""
    try:
        src = subprocess.run(["git", "show", "main:ros2_ws/src/gurdy_safety/gurdy_safety/safety_shield_node.py"],
                             cwd=ROOT, capture_output=True, text=True, check=True).stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
        pytest.skip("no git / no main branch here")
    d = tmp_path / "ros2_ws/src/gurdy_safety/gurdy_safety"
    d.mkdir(parents=True)
    (d / "safety_shield_node.py").write_text(src)
    r = subprocess.run(["patch", "-p1", "--dry-run", "-i", str(ROOT / "docs/results/data/phase9f/shield_scan_timeout.patch")],
                       cwd=tmp_path, capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
