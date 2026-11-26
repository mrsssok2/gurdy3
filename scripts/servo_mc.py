#!/usr/bin/env python3
"""Phase 9e: Monte-Carlo of the visual servo (synthetic wrist depth, kinematic robot) -> markdown table.

    python3 scripts/servo_mc.py [--n 60]      # numpy + pyyaml; no ROS / Gazebo
Start error uniform in the Nav2 xy tolerance disc (r <= 0.12 m), object yaw random, step gain 0.85-1.15 + 8 mm noise,
depth noise 0.003 z^2. 'true error' = ground-truth object position vs the grasp-zone centre at the end.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "gurdy_core"), str(ROOT / "gurdy_core/tests")]

from gurdy_core import arm as A  # noqa: E402
from gurdy_core import visual_servo as VS  # noqa: E402
from servo_sim import SimRobot  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=60)
    a_ = ap.parse_args()
    arm, rb = A.load_all()
    gz = A.ground_z(rb)
    ok_fn = lambda c, xy: A.plan_grasp(arm, rb, A.PICKABLE[c], xy, gz) is not None   # noqa: E731
    print("| class | trials | success | ALIGNED / ALIGNED_ARM | failures | true final error median / max (cm) | grasp feasible at the end | steps mean | time mean (s) |")
    print("|---|---|---|---|---|---|---|---|---|")
    rng = np.random.default_rng(0)
    for cls, (_, _, _, t) in VS.CLASS_INFO.items():
        codes, errs, feas, steps, times = [], [], [], [], []
        for k in range(a_.n):
            r_, th = 0.12 * math.sqrt(rng.random()), rng.uniform(0, 2 * math.pi)
            start = (t[0] + r_ * math.cos(th), t[1] + r_ * math.sin(th))
            rob = SimRobot(cls, start, yaw_obj=float(rng.uniform(-3, 3)), step_gain=(0.85, 1.15), step_noise=0.008, seed=k,
                           depth_noise=0.003)
            res = VS.VisualServo(rob, arm_reachable=ok_fn).run(cls, prior_xy=start)
            codes.append(res.code)
            if res.ok:
                errs.append(float(np.hypot(*(rob.obj_rel() - np.array(t)))))
                feas.append(ok_fn(cls, rob.obj_rel()))
            steps.append(res.steps)
            times.append(rob.now())
        n_ok = sum(c.startswith("ALIGNED") for c in codes)
        fails = {c: codes.count(c) for c in set(codes) if not c.startswith("ALIGNED")}
        print(f"| {cls} | {a_.n} | {100 * n_ok / a_.n:.0f} % | {codes.count('ALIGNED')} / {codes.count('ALIGNED_ARM')} | "
              f"{fails or '-'} | {np.median(errs) * 100:.1f} / {max(errs) * 100:.1f} | {sum(feas)}/{len(feas)} | "
              f"{np.mean(steps):.1f} | {np.mean(times):.0f} |")


if __name__ == "__main__":
    main()
