#!/usr/bin/env python3
"""Record an exploration run (sim time): coverage vs time, ground-truth path, stuck events, map snapshots for a time-lapse.
    python3 scripts/explore_monitor.py OUT_DIR DONE_FILE [--interval 5] [--snap 10]
Writes OUT_DIR/timeseries.csv (t, known_m2, interior_fraction, free_m2, path_m, speed), OUT_DIR/snaps/NNNN.npz (map + pose),
OUT_DIR/monitor.json (final numbers). Stops 5 sim s after DONE_FILE appears (the explorer writes it) or on SIGTERM.
Stuck event = ground-truth speed < 0.02 m/s for > 10 s sim while exploring (before DONE_FILE); counted once per episode.
Needs /odom/ground_truth (map frame == world frame: the robot spawns at the world origin).
"""
import argparse
import json
import math
import os
import signal
import sys
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gurdy_core"))
from gurdy_core import frontier as fr   # noqa: E402

WORLD = (-2.0, 6.5, -3.0, 3.0)          # flat_lab interior bounds (gen_world_flat_lab.py)
if os.environ.get("WORLD_BOUNDS"):       # xmin,xmax,ymin,ymax (lab4: -2,6,-2,6)
    WORLD = tuple(float(v) for v in os.environ["WORLD_BOUNDS"].split(","))


class Mon(Node):
    def __init__(self, out, done, interval, snap):
        super().__init__("explore_monitor")
        self.out, self.done, self.interval, self.snap_dt = Path(out), done, interval, snap
        (self.out / "snaps").mkdir(parents=True, exist_ok=True)
        self.csv = open(self.out / "timeseries.csv", "w")
        self.csv.write("t_s,known_m2,interior_fraction,free_m2,path_m,speed_mps,x,y\n")
        self.t0 = self.now = None
        self.map = None
        self.pose = None
        self.path = 0.0
        self.prev = None
        self.speed = 0.0
        self.last_row = self.last_snap = -1e9
        self.still_since, self.stuck_events, self.in_stuck, self.max_still = None, 0, False, 0.0
        self.done_t = None
        self.nsnap = 0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.create_subscription(Odometry, "/odom/ground_truth", self.on_odom, 20)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.last_odom_t, self.last_pos = None, None

    def on_map(self, m):
        self.map = (np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width), m.info.resolution,
                    (m.info.origin.position.x, m.info.origin.position.y))

    def on_odom(self, m):
        p = m.pose.pose.position
        if self.prev is not None:
            self.path += math.hypot(p.x - self.prev[0], p.y - self.prev[1])
        self.prev = (p.x, p.y)
        self.pose = (p.x, p.y)

    def on_clock(self, c):
        self.now = c.clock.sec + c.clock.nanosec * 1e-9
        if self.t0 is None and self.map is not None and self.pose is not None:
            self.t0 = self.now
        if self.t0 is None:
            return
        t = self.now - self.t0
        # 1 s speed windows + stuck detection
        if self.last_odom_t is None:
            self.last_odom_t, self.last_pos = self.now, self.pose
        elif self.now - self.last_odom_t >= 1.0:
            self.speed = math.hypot(self.pose[0] - self.last_pos[0], self.pose[1] - self.last_pos[1]) / (self.now - self.last_odom_t)
            self.last_odom_t, self.last_pos = self.now, self.pose
            if self.done_t is None:
                if self.speed < 0.02:
                    self.still_since = self.still_since if self.still_since is not None else self.now
                    self.max_still = max(self.max_still, self.now - self.still_since)
                    if self.now - self.still_since > 10.0 and not self.in_stuck:
                        self.stuck_events += 1
                        self.in_stuck = True
                else:
                    self.still_since, self.in_stuck = None, False
        if t - self.last_row >= self.interval:
            self.last_row = t
            grid, res, org = self.map
            s = fr.map_stats(grid, res, org, WORLD)
            self.csv.write(f"{t:.1f},{s['known_m2']:.2f},{s['interior_fraction']:.4f},{s['free_m2']:.2f},{self.path:.2f},"
                           f"{self.speed:.3f},{self.pose[0]:.2f},{self.pose[1]:.2f}\n")
            self.csv.flush()
            self.final = s
        if t - self.last_snap >= self.snap_dt:
            self.last_snap = t
            self.save_snap(t)
        if self.done_t is None and os.path.exists(self.done):
            self.done_t = self.now
            self.save_snap(t)
        if self.done_t is not None and self.now - self.done_t >= 5.0:
            self.finish()

    def save_snap(self, t):
        grid, res, org = self.map
        np.savez_compressed(self.out / "snaps" / f"{self.nsnap:04d}.npz", grid=grid, res=res, origin=np.array(org),
                            t=t, pose=np.array(self.pose), path=self.path)
        self.nsnap += 1

    def finish(self):
        grid, res, org = self.map
        s = fr.map_stats(grid, res, org, WORLD)
        s.update({"sim_time_s": round(self.now - self.t0, 1), "gt_path_m": round(self.path, 2), "stuck_events": self.stuck_events,
                  "max_still_s": round(self.max_still, 1), "snapshots": self.nsnap, "end_pose": list(self.pose)})
        (self.out / "monitor.json").write_text(json.dumps(s, indent=1))
        self.csv.close()
        self.get_logger().info("monitor done")
        raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out"); ap.add_argument("done")
    ap.add_argument("--interval", type=float, default=5.0); ap.add_argument("--snap", type=float, default=10.0)
    a = ap.parse_args()
    rclpy.init()
    n = Mon(a.out, a.done, a.interval, a.snap)
    signal.signal(signal.SIGTERM, lambda *_: n.finish() if n.t0 is not None else sys.exit(1))
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    main()
