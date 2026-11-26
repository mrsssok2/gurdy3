#!/usr/bin/env python3
"""Phase 8b: run every safety-shield scenario in the 2D kinematic simulator, with and without the shield -> markdown.

    python3 scripts/shield_sim_report.py            # stdlib only, ~1 min on one core; no ROS / Gazebo / Docker

Nominal robot model: command latency 0.05 s, velocity lag tau 0.2 s, noiseless lidar. Robustness sweep: latency 0.1 s,
tau 0.4 s, lidar noise 0.01 m (robot.yaml), 3 seeds.
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gurdy_core"))

from gurdy_core.safety_shield import SafetyShield, scan_to_points  # noqa: E402
from gurdy_core.shield_scenarios import (DOOR, FOOTPRINT, distance_after, doorway_pass_time, run_scenario,  # noqa: E402
                                         scenarios, shield_config)
from gurdy_core.shield_sim import raycast  # noqa: E402


def f(v, nd=3):
    return "-" if v is None or not math.isfinite(v) else f"{v:.{nd}f}"


def main():
    sc = scenarios()
    print("| scenario | what | collision without | collision with | min clearance without (m) | min clearance with (m) "
          "| path without / with (m) | sweep: collisions / runs, worst min clearance |")
    print("|---|---|---|---|---|---|---|---|")
    total_coll, total_runs = 0, 0
    for name, s in sc.items():
        a, b = run_scenario(name, False), run_scenario(name, True)
        sweep = [run_scenario(name, True, latency=0.1, tau=0.4, noise=0.01, seed=k) for k in range(3)]
        nc = sum(r.collided for r in sweep) + b.collided
        total_coll += nc
        total_runs += len(sweep) + 1
        print(f"| `{name}` | {s['note']} | {'YES' if a.collided else 'no'} | {'YES' if b.collided else 'no'} "
              f"| {f(a.min_clearance)} | {f(b.min_clearance)} | {a.path_len:.2f} / {b.path_len:.2f} "
              f"| {sum(r.collided for r in sweep)} / 3, {f(min(r.min_clearance for r in sweep))} |")
    print(f"\nShielded collisions: {total_coll} / {total_runs} runs (nominal + sweep).\n")

    print("| doorway scenario | time to cross x = 2.5 without (s) | with (s) | slowdown | goal reached with | "
          "scale_only: crossed? |")
    print("|---|---|---|---|---|---|")
    for name in ("doorway_centre", "doorway_offset", "doorway_diagonal"):
        a, b = run_scenario(name, False), run_scenario(name, True)
        c = run_scenario(name, True, scale_only=True)
        ta, tb, tc = doorway_pass_time(a), doorway_pass_time(b), doorway_pass_time(c)
        print(f"| `{name}` | {f(ta, 2)} | {f(tb, 2)} | {100 * (tb / ta - 1):+.1f} % | {b.reached} | "
              f"{'yes, ' + f(tc, 2) + ' s' if tc else 'NO (stalls at the jamb)'} |")

    r = run_scenario("stale_scan", True)
    tz = next(t for t, *_x, st in r.trace if t > 2.0 and st == "stale")
    print(f"\nstale_scan: last scan 1.9 s, shield output zero from t = {tz:.2f} s, distance driven after that: "
          f"{distance_after(r, tz):.3f} m (velocity lag); no_scan: path {run_scenario('no_scan', True).path_len:.3f} m.")

    # compute time of one scan update + filter, worst case = in the doorway (many sectors bind)
    sh = SafetyShield(shield_config())
    rr = raycast((2.5, 0.35, 0.0), DOOR)
    t0 = time.perf_counter()
    n = 200
    for _ in range(n):
        sh.update_laserscan(rr, -math.pi, 2 * math.pi / 360, 0.15, 8.0, 0.0)
        sh.filter((0.15, 0.0, 0.3), 0.0)
    dt = (time.perf_counter() - t0) / n
    print(f"compute (this host, CPython {sys.version.split()[0]}): {1e3 * dt:.2f} ms per scan update + filter in the doorway "
          f"({len(scan_to_points(rr, -math.pi, 2 * math.pi / 360, 0.15, 8.0))} points, {len(sh.cons)} constraints).")


if __name__ == "__main__":
    main()
