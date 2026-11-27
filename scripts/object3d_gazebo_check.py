#!/usr/bin/env python3
"""Phase 8c Gazebo accuracy check, ROS side (NOT RUN YET). Run by scripts/object3d_gazebo_check.sh, inside the sim container,
with semantic_lab + detector_node (object3d:=true, target_frame:=odom, detector_log) + odom_tf_node already up.

    python3 scripts/object3d_gazebo_check.py RUN_DIR [--only NAME ...] [--settle 3] [--hold 2]

For each stop of vision/accuracy3d.plan_stops (11 objects x 4 ranges x 3 bearings = 132):
  1. teleport the robot (gz service /world/semantic_lab/set_pose, model `gurdy`, z = stance height + 5 mm, level)
  2. wait `settle` s of SIM time (from /odom/ground_truth stamps), then record [t0, t1] = `hold` s of standing still
  3. check the ground-truth pose over the window: xy within 5 cm / yaw within 3 deg of the plan, |roll|, |pitch| < 0.25 rad
     (not fallen), xy drift < 2 cm -> `valid`; else one retry, then marked invalid with the reason
stops.json gets t0 / t1 (sim time, same clock as the detector's image stamps), the mean GT pose and the checks; the
scorer (vision/accuracy3d.py score RUN_DIR) matches sightings to stops by time.

Assumption to verify on the first stop (runbook in docs/results/phase8c_plan.md): with ground-truth odometry and the
spawn at the origin, odom = world in x / y / yaw after a teleport (gz OdometryPublisher integrates pose differences). The
check in 3 compares the GT odometry with the commanded world pose, so a broken assumption shows as invalid stops, not
as silently wrong errors.
"""
import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "vision"))
sys.path.insert(0, str(ROOT / "gurdy_core"))
import accuracy3d as A  # noqa: E402

WORLD, MODEL = "semantic_lab", "gurdy"
BASE_Z = 0.115                   # robot.stance body_height 0.11 + 5 mm, as the spawn in gurdy_gazebo/sim.launch.py


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def roll_pitch(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


def teleport(x, y, yaw, timeout_ms=3000) -> bool:
    req = (f'name: "{MODEL}" position {{x: {x:.4f} y: {y:.4f} z: {BASE_Z:.4f}}} '
           f'orientation {{x: 0 y: 0 z: {math.sin(yaw / 2):.6f} w: {math.cos(yaw / 2):.6f}}}')
    r = subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "gz.msgs.Pose",
                        "--reptype", "gz.msgs.Boolean", "--timeout", str(timeout_ms), "--req", req],
                       capture_output=True, text=True, timeout=timeout_ms / 1000 + 5)
    return r.returncode == 0 and "data: true" in r.stdout


class GT(Node):
    def __init__(self):
        super().__init__("object3d_gazebo_check")
        self.last = None
        self.buf = []
        self.create_subscription(Odometry, "/odom/ground_truth", self.on_odom, qos_profile_sensor_data)

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.pose.position, m.pose.pose.orientation
        r, pi = roll_pitch(q)
        self.last = (t, p.x, p.y, p.z, yaw_of(q), r, pi)
        self.buf.append(self.last)

    def now(self):
        return self.last[0] if self.last else None

    def wait_sim(self, dt, wall_timeout=60.0) -> bool:
        while self.now() is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        t_end, w_end = self.now() + dt, time.time() + wall_timeout
        while self.now() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() > w_end:
                return False
        return True


def run_stop(node: GT, st: dict, settle: float, hold: float) -> dict:
    if not teleport(st["x"], st["y"], st["yaw"]):
        return {"valid": False, "reason": "set_pose failed"}
    if not node.wait_sim(settle):
        return {"valid": False, "reason": "sim clock stalled (settle)"}
    node.buf = []
    t0 = node.now()
    if not node.wait_sim(hold):
        return {"valid": False, "reason": "sim clock stalled (hold)"}
    t1 = node.now()
    w = [b for b in node.buf if t0 <= b[0] <= t1] or [node.last]
    mx, my = sum(b[1] for b in w) / len(w), sum(b[2] for b in w) / len(w)
    myaw = math.atan2(sum(math.sin(b[4]) for b in w), sum(math.cos(b[4]) for b in w))
    drift = math.hypot(w[-1][1] - w[0][1], w[-1][2] - w[0][2])
    tilt = max(max(abs(b[5]), abs(b[6])) for b in w)
    exy = math.hypot(mx - st["x"], my - st["y"])
    eyaw = abs(math.atan2(math.sin(myaw - st["yaw"]), math.cos(myaw - st["yaw"])))
    reason = ("fallen / tilted" if tilt > 0.25 else "pose off plan (odom != world?)" if exy > 0.05 or eyaw > math.radians(3)
              else "moving" if drift > 0.02 else "")
    return {"valid": not reason, "reason": reason, "t0": t0, "t1": t1, "gt_pose": [round(mx, 4), round(my, 4), round(myaw, 4)],
            "pose_err_xy": round(exy, 4), "pose_err_yaw_deg": round(math.degrees(eyaw), 2), "drift": round(drift, 4),
            "max_tilt": round(tilt, 3), "n_odom": len(w)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--only", nargs="*", help="GT object names (default: all)")
    ap.add_argument("--settle", type=float, default=3.0, help="sim s after the teleport before recording")
    ap.add_argument("--hold", type=float, default=2.0, help="sim s of standing still per stop")
    a = ap.parse_args()
    run = Path(a.run_dir)
    run.mkdir(parents=True, exist_ok=True)
    plan = A.plan_stops()
    stops = [s for s in plan["stops"] if not a.only or s["name"] in a.only]
    rclpy.init()
    node = GT()
    out = []
    t_wall = time.time()
    try:
        for i, st in enumerate(stops):
            res = run_stop(node, st, a.settle, a.hold)
            if not res["valid"]:
                res = {**run_stop(node, st, a.settle, a.hold), "retried": True}
            out.append({**st, **res})
            print(f"[{i + 1}/{len(stops)}] {st['id']}: {'ok' if res['valid'] else 'INVALID ' + res['reason']} "
                  f"(pose err {res.get('pose_err_xy')} m, {res.get('pose_err_yaw_deg')} deg)", flush=True)
            (run / "stops.json").write_text(json.dumps({"stops": out, "skipped": plan["skipped"],
                                                         "settle": a.settle, "hold": a.hold}, indent=1) + "\n")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    n_ok = sum(s["valid"] for s in out)
    print(f"DONE {n_ok}/{len(out)} valid stops, {time.time() - t_wall:.0f} s wall", flush=True)
    return 0 if n_ok else 1


if __name__ == "__main__":
    sys.exit(main())
