#!/usr/bin/env python3
"""Phase 8b live probe (Gazebo, lab4, shield:=true): drive the robot with direct /cmd_vel (teleop path, no collision monitor) and measure,
from GROUND TRUTH (/odom/ground_truth + the lab4 wall boxes of scripts/gen_world_lab4.py), the clearance between the robot and the walls.
    python3 scripts/shield_probe.py OUT.json wall|corner_turn|stale
Clearances: `feet` = convex hull of the feet incl. foot radius (footprint_polygon margin 0; touching = <= 0), `nav` = the Nav2 / shield
footprint (margin 0.04, the polygon the shield keeps d_safe away from). Swinging legs can reach outside the feet hull (not modelled).
Velocity in the record = finite difference of GT positions (0.25 s window)."""
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
import gen_world_lab4 as lab4  # noqa: E402
from gurdy_core.robot import load_robot  # noqa: E402
from gurdy_navigation.footprint import footprint_polygon  # noqa: E402

ROBOT = load_robot()
FEET = [tuple(p) for p in footprint_polygon(ROBOT, 0.0)]
NAV = [tuple(p) for p in footprint_polygon(ROBOT, 0.04)]
RECTS = [(cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2) for _, (cx, cy), (sx, sy) in lab4.walls()]


def _seg_pt(a, b, p):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)))
    return math.hypot(a[0] + t * dx - p[0], a[1] + t * dy - p[1])


def _inside(poly, p):
    s = None
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        c = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if s is None:
            s = c > 0
        elif (c > 0) != s:
            return False
    return True


def clearance(poly_body, x, y, yaw):
    """Min distance between the robot polygon (world) and every wall box; <= 0 = overlap."""
    c, s = math.cos(yaw), math.sin(yaw)
    poly = [(x + c * px - s * py, y + s * px + c * py) for px, py in poly_body]
    best = 1e9
    for x0, y0, x1, y1 in RECTS:
        rc = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        if any(x0 <= p[0] <= x1 and y0 <= p[1] <= y1 for p in poly) or any(_inside(poly, q) for q in rc):
            return 0.0
        for p in poly:
            for i in range(4):
                best = min(best, _seg_pt(rc[i], rc[(i + 1) % 4], p))
        for q in rc:
            for i in range(len(poly)):
                best = min(best, _seg_pt(poly[i], poly[(i + 1) % len(poly)], q))
    return best


class Probe(Node):
    def __init__(self):
        super().__init__("shield_probe")
        self.pose, self.out, self.status, self.scan_t = None, None, {}, None
        self.hist = []
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom/ground_truth", self.on_odom, 50)
        self.create_subscription(Twist, "/cmd_vel_shielded", lambda m: setattr(self, "out", (m.linear.x, m.linear.y, m.angular.z)), 50)
        self.create_subscription(String, "/shield/status", lambda m: setattr(self, "status", json.loads(m.data)), 20)
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "scan_t", time.monotonic()), qos_profile_sensor_data)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.hist.append((time.monotonic(), p.x, p.y))
        self.hist = self.hist[-200:]

    def speed(self):
        h = self.hist
        if len(h) < 2:
            return 0.0
        t1 = h[-1][0]
        old = next((e for e in h if t1 - e[0] <= 0.25), h[0])
        dt = t1 - old[0]
        return math.hypot(h[-1][1] - old[1], h[-1][2] - old[2]) / dt if dt > 0.05 else 0.0

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def drive(self, cmd, dur, rec, label, hook=None):
        t0 = time.monotonic()
        nxt = t0
        while time.monotonic() - t0 < dur:
            if time.monotonic() >= nxt:
                tw = Twist()
                tw.linear.x, tw.linear.y, tw.angular.z = cmd
                self.pub.publish(tw)
                nxt += 0.05
                if self.pose:
                    x, y, yaw = self.pose
                    rec.append({"phase": label, "t": round(time.monotonic() - t0, 3), "x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4),
                                "v": round(self.speed(), 4), "feet": round(clearance(FEET, x, y, yaw), 4), "nav": round(clearance(NAV, x, y, yaw), 4),
                                "out": [round(v, 4) for v in self.out] if self.out else None, "state": self.status.get("state"),
                                "shield_min": self.status.get("min_clearance"),
                                "scan_age": round(time.monotonic() - self.scan_t, 3) if self.scan_t else None})
                if hook:
                    hook(time.monotonic() - t0)
            rclpy.spin_once(self, timeout_sec=0.005)
        self.pub.publish(Twist())


def summary(rec, label):
    r = [e for e in rec if e["phase"] == label]
    if not r:
        return {}
    end = r[-1]
    return {"phase": label, "n": len(r), "min_feet_clearance": min(e["feet"] for e in r), "min_nav_clearance": min(e["nav"] for e in r),
            "end_feet_clearance": end["feet"], "end_nav_clearance": end["nav"], "end_pose": [end["x"], end["y"], end["yaw"]],
            "max_speed": max(e["v"] for e in r), "end_speed": end["v"], "touched": any(e["feet"] <= 0.0 for e in r),
            "states": sorted({str(e["state"]) for e in r})}


def main():
    out, mode = sys.argv[1], sys.argv[2]
    rclpy.init()
    n = Probe()
    n.spin_for(3.0)
    rec, res = [], {"mode": mode, "start_pose": n.pose}
    if mode == "wall":          # robot near (0, 0.5) facing +x; divider x = 2 (solid for y in [-0.2, 2]) ahead
        n.drive((0.15, 0.0, 0.0), 30.0, rec, "wall")
        res["summary"] = [summary(rec, "wall")]
    elif mode == "corner_turn":  # robot near (-0.1, 4.5) facing ~+x in room C; NE corner (2, 6) ahead-left
        n.drive((0.15, 0.10, 0.0), 30.0, rec, "corner")
        n.drive((0.0, 0.0, 0.6), 15.0, rec, "turn_left")
        n.drive((0.0, 0.0, -0.6), 15.0, rec, "turn_right")
        res["summary"] = [summary(rec, k) for k in ("corner", "turn_left", "turn_right")]
    elif mode == "stale":        # drive slowly into free space, kill the /scan producer at 4 s, restart it at 8 s
        ev = {}

        def hook(t):
            if "kill" not in ev and t >= 4.0:
                ev["kill"] = round(t, 3)
                subprocess.run(["pkill", "-f", "[s]can_self_filter_node"])
            if "restart" not in ev and t >= 8.0:
                ev["restart"] = round(t, 3)
                ev["proc"] = subprocess.Popen(["ros2", "run", "gurdy_navigation", "scan_self_filter_node"], stdout=subprocess.DEVNULL,
                                              stderr=subprocess.DEVNULL)
        n.drive((-0.10, 0.0, 0.0), 14.0, rec, "stale", hook)
        r = [e for e in rec if e["phase"] == "stale"]
        k = ev["kill"]
        zero = next((e["t"] for e in r if e["t"] > k and e["out"] is not None and all(abs(v) < 1e-9 for v in e["out"])), None)
        still = next((e["t"] for e in r if zero and e["t"] > zero and e["v"] < 0.01), None)
        moving_again = next((e["t"] for e in r if e["t"] > ev["restart"] and e["out"] and abs(e["out"][0]) > 0.05), None)
        res["summary"] = [summary(rec, "stale")]
        res["stale"] = {"kill_t": k, "restart_t": ev["restart"], "first_zero_out_t": zero, "zero_after_kill_s": round(zero - k, 3) if zero else None,
                        "robot_still_t": still, "out_moving_again_t": moving_again,
                        "states_after_kill": sorted({str(e["state"]) for e in r if k < e["t"] < ev["restart"]})}
        ev.pop("proc", None)
    res["records"] = rec
    json.dump(res, open(out, "w"), indent=1)
    print(json.dumps(res["summary"] + ([res["stale"]] if "stale" in res else []), indent=1))
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
