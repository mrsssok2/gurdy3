#!/usr/bin/env python3
"""Probes for scripts/day_0927_checks.sh (run INSIDE the sim container with the stack up; rclpy). NOT run yet.

    day0927_probe.py arm_state OUT.json            # arm joints in /joint_states, close to `folded`; robot standing (GT odom z)
    day0927_probe.py walk OUT.json [seconds]       # /cmd_vel 0.10 m/s forward then 0.3 rad/s turn; GT-odom tracking + falls
    day0927_probe.py moveit OUT.json               # MoveGroup action, plan only: folded -> servo_view -> pre_grasp (analytic IK)
    day0927_probe.py wrist OUT.json                # arm to servo_view, trigger the wrist camera, frames + K + fitted floor height
Each prints one line 'PASS ...' or 'FAIL ...' and writes the numbers to OUT.json. Exit code 0 = PASS.
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "gurdy_core"))
from gurdy_core import arm as A  # noqa: E402

ARM = A.load_arm()


class Probe(Node):
    def __init__(self):
        super().__init__("day0927_probe")
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        self.js, self.odom = {}, []
        self.create_subscription(JointState, "/joint_states", lambda m: self.js.update(dict(zip(m.name, m.position))), 10)
        self.create_subscription(Odometry, "/odom/ground_truth", self._odom, 10)

    def _odom(self, m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom.append((time.monotonic(), p.x, p.y, p.z, yaw, v.linear.x, v.angular.z))

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def done(ok, msg, out, data):
    Path(out).write_text(json.dumps({"pass": bool(ok), "msg": msg, **data}, indent=1, default=float))
    print(("PASS " if ok else "FAIL ") + msg)
    sys.exit(0 if ok else 1)


def arm_state(p, out):
    p.spin_for(5.0)
    q = [p.js.get(n) for n in A.ARM_JOINTS]
    if any(v is None for v in q):
        done(False, f"arm joints missing in /joint_states: {[n for n, v in zip(A.ARM_JOINTS, q) if v is None]}", out, {})
    err = float(np.max(np.abs(np.array(q) - ARM.poses["folded"])))
    z = p.odom[-1][3] if p.odom else float("nan")
    ok = err < 0.05 and z > 0.07
    done(ok, f"arm max error to folded {err:.3f} rad, base z {z:.3f} m", out, {"q": q, "err": err, "z": z})


def walk(p, out, seconds=30.0):
    from geometry_msgs.msg import Twist
    pub = p.create_publisher(Twist, "/cmd_vel", 10)
    res = {}
    for name, (vx, wz) in (("forward", (0.10, 0.0)), ("turn", (0.0, 0.3))):
        p.odom.clear()
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            m = Twist()
            m.linear.x, m.angular.z = vx, wz
            pub.publish(m)
            p.spin_for(0.05)
        pub.publish(Twist())
        o = np.array(p.odom[len(p.odom) // 4:])            # skip the start-up quarter
        if len(o) < 10:
            done(False, f"{name}: no ground-truth odometry", out, res)
        meas = float(np.mean(o[:, 5])) if vx else float(np.mean(o[:, 6]))
        cmd = vx or wz
        res[name] = {"cmd": cmd, "measured": meas, "err_pct": 100 * abs(meas - cmd) / cmd, "min_z": float(np.min(o[:, 3]))}
        p.spin_for(3.0)
    fell = any(r["min_z"] < 0.06 for r in res.values())
    msg = ", ".join(f"{k} {v['err_pct']:.0f} % err" for k, v in res.items()) + (", FELL" if fell else ", no fall")
    done(not fell, msg + " (compare with the arm:=false run: acceptance <= +5 points)", out, res)


def moveit(p, out):
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
    from rclpy.action import ActionClient
    ac = ActionClient(p, MoveGroup, "/move_action")
    if not ac.wait_for_server(timeout_sec=30.0):
        done(False, "no /move_action server (move_group not running)", out, {})
    rb = A.load_robot()
    gz = A.ground_z(rb)
    pg = A.plan_grasp(ARM, rb, A.PICKABLE["small_box"], (-0.27, 0.0), gz)
    targets = {"servo_view": ARM.poses["servo_view"], "pre_grasp": pg[0] if pg else None}
    res = {}
    for name, q in targets.items():
        if q is None:
            done(False, "no analytic pre-grasp", out, res)
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        c = Constraints()
        for n, v in zip(A.ARM_JOINTS, q):
            c.joint_constraints.append(JointConstraint(joint_name=n, position=float(v), tolerance_above=0.01,
                                                       tolerance_below=0.01, weight=1.0))
        goal.request.goal_constraints.append(c)
        goal.planning_options.plan_only = True
        f = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(p, f, timeout_sec=10.0)
        h = f.result()
        if h is None or not h.accepted:
            done(False, f"{name}: goal rejected", out, res)
        r = h.get_result_async()
        rclpy.spin_until_future_complete(p, r, timeout_sec=30.0)
        code = r.result().result.error_code.val if r.result() else None
        res[name] = {"error_code": code, "planning_time": r.result().result.planning_time if r.result() else None}
        if code != MoveItErrorCodes.SUCCESS:
            done(False, f"{name}: plan failed, MoveIt error code {code}", out, res)
    done(True, "plans folded -> servo_view and -> pre_grasp found (plan only)", out, res)


def wrist(p, out):
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Bool
    from trajectory_msgs.msg import JointTrajectoryPoint
    from gurdy_core import low_obstacles as LO
    ac = ActionClient(p, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    if not ac.wait_for_server(timeout_sec=30.0):
        done(False, "no arm_controller action", out, {})
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = A.ARM_JOINTS
    pt = JointTrajectoryPoint(positions=[float(v) for v in ARM.poses["servo_view"]])
    pt.time_from_start.sec = 4
    g.trajectory.points.append(pt)
    f = ac.send_goal_async(g)
    rclpy.spin_until_future_complete(p, f, timeout_sec=10.0)
    p.spin_for(6.0)
    frames, info = [], []
    p.create_subscription(Image, "/wrist_camera/depth_image", frames.append, 2)
    p.create_subscription(CameraInfo, "/wrist_camera/camera_info", info.append, 2)
    trig = p.create_publisher(Bool, "/wrist_camera/trigger", 2)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        trig.publish(Bool(data=True))
        p.spin_for(0.2)
    if not frames or not info:
        done(False, f"wrist camera: {len(frames)} depth frames, {len(info)} camera_info (trigger / bridge?)", out, {})
    m, k = frames[-1], info[-1].k
    d = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
    K = (k[0], k[4], k[2], k[5])
    fx_cfg = (ARM.cam["width"] / 2) / math.tan(ARM.cam["hfov"] / 2)
    q = [p.js.get(n, v) for n, v in zip(A.ARM_JOINTS, ARM.poses["servo_view"])]
    T = A.T_base_wrist_optical(ARM, q)
    pts, _ = LO.depth_to_points(d, K, LO.wrist_params())
    n, dd, fitted, _ = LO.fit_floor(LO.apply_T(T, pts), LO.wrist_params())
    floor_z = -dd / n[2] if fitted else float("nan")
    ok = fitted and abs(K[0] - fx_cfg) < 1.0 and abs(floor_z + 0.11) < 0.02
    done(ok, f"{len(frames)} frames in 5 s, fx {K[0]:.1f} (arm.yaml {fx_cfg:.1f}), fitted floor z {floor_z:.3f} m in base_link "
             f"(expected about -0.11)", out, {"frames": len(frames), "K": K, "floor_z": floor_z, "nan_share": float(np.isnan(d).mean())})


def main():
    cmd, out = sys.argv[1], sys.argv[2]
    rclpy.init()
    p = Probe()
    try:
        {"arm_state": lambda: arm_state(p, out), "walk": lambda: walk(p, out, float(sys.argv[3]) if len(sys.argv) > 3 else 30.0),
         "moveit": lambda: moveit(p, out), "wrist": lambda: wrist(p, out)}[cmd]()
    finally:
        p.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
