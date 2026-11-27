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
    """Plan only, chained: folded (current) -> servo_view, then servo_view (end of plan 1) -> pre_grasp. Every waypoint
    of both trajectories is checked with /check_state_validity (full robot state incl. the legs from /joint_states,
    MoveIt's description has the leg collision proxies) and contacts with leg links are reported."""
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
    from moveit_msgs.srv import GetStateValidity
    from rclpy.action import ActionClient
    from sensor_msgs.msg import JointState
    ac = ActionClient(p, MoveGroup, "/move_action")
    if not ac.wait_for_server(timeout_sec=60.0):
        done(False, "no /move_action server (move_group not running)", out, {})
    sv = p.create_client(GetStateValidity, "/check_state_validity")
    sv.wait_for_service(timeout_sec=30.0)
    p.spin_for(2.0)
    rb = A.load_robot()
    gz = A.ground_z(rb)
    pg = A.plan_grasp(ARM, rb, A.PICKABLE["small_box"], (-0.27, 0.0), gz)
    targets = {"servo_view": ARM.poses["servo_view"], "pre_grasp": pg[0] if pg else None}
    base_js = {k: v for k, v in p.js.items()}
    res, start = {}, None
    for name, q in targets.items():
        if q is None:
            done(False, "no analytic pre-grasp", out, res)
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        if start is not None:
            js = JointState(name=list(base_js), position=[float(v) for v in base_js.values()])
            for n, v in zip(A.ARM_JOINTS, start):
                js.position[js.name.index(n)] = float(v)
            goal.request.start_state = RobotState(joint_state=js, is_diff=False)
        c = Constraints()
        for n, v in zip(A.ARM_JOINTS, q):
            c.joint_constraints.append(JointConstraint(joint_name=n, position=float(v), tolerance_above=0.01,
                                                       tolerance_below=0.01, weight=1.0))
        goal.request.goal_constraints.append(c)
        goal.planning_options.plan_only = True
        t0 = time.monotonic()
        f = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(p, f, timeout_sec=10.0)
        h = f.result()
        if h is None or not h.accepted:
            done(False, f"{name}: goal rejected", out, res)
        r = h.get_result_async()
        rclpy.spin_until_future_complete(p, r, timeout_sec=60.0)
        wall = time.monotonic() - t0
        rr = r.result().result if r.result() else None
        code = rr.error_code.val if rr else None
        res[name] = {"error_code": code, "planning_time_s": rr.planning_time if rr else None, "wall_s": wall}
        if code != MoveItErrorCodes.SUCCESS:
            done(False, f"{name}: plan failed, MoveIt error code {code}", out, res)
        jt = rr.planned_trajectory.joint_trajectory
        n_pts, bad, leg_contacts = len(jt.points), 0, set()
        for pt in jt.points:
            js = JointState(name=list(base_js), position=[float(v) for v in base_js.values()])
            for n, v in zip(jt.joint_names, pt.positions):
                if n in js.name:
                    js.position[js.name.index(n)] = float(v)
                else:
                    js.name.append(n); js.position.append(float(v))
            req = GetStateValidity.Request(robot_state=RobotState(joint_state=js), group_name="arm")
            fv = sv.call_async(req)
            rclpy.spin_until_future_complete(p, fv, timeout_sec=5.0)
            v = fv.result()
            if v is None:
                continue
            if not v.valid:
                bad += 1
            for ct in v.contacts:
                pair = tuple(sorted((ct.contact_body_1, ct.contact_body_2)))
                if any(("coxa" in b or "femur" in b or "tibia" in b or "foot" in b or "leg" in b) for b in pair):
                    leg_contacts.add(pair)
        res[name].update({"waypoints": n_pts, "invalid_waypoints": bad, "leg_contacts": sorted(leg_contacts),
                          "end": list(jt.points[-1].positions) if jt.points else None,
                          "joint_names": list(jt.joint_names), "points": [list(pt.positions) for pt in jt.points]})
        start = [jt.points[-1].positions[jt.joint_names.index(n)] for n in A.ARM_JOINTS] if jt.points else q
    # sanity: random arm poses through the same service; the checker must flag some leg contacts or it proves nothing
    rng = np.random.default_rng(0)
    lo = np.array([ARM.limits[n][0] for n in A.ARM_JOINTS]); hi = np.array([ARM.limits[n][1] for n in A.ARM_JOINTS])
    n_inv = n_leg = 0
    for _ in range(200):
        qr = rng.uniform(lo, hi)
        js = JointState(name=list(base_js), position=[float(v) for v in base_js.values()])
        for n, v in zip(A.ARM_JOINTS, qr):
            js.position[js.name.index(n)] = float(v)
        fv = sv.call_async(GetStateValidity.Request(robot_state=RobotState(joint_state=js), group_name="arm"))
        rclpy.spin_until_future_complete(p, fv, timeout_sec=5.0)
        v = fv.result()
        if v is not None and not v.valid:
            n_inv += 1
            if any("proxy" in c.contact_body_1 or "proxy" in c.contact_body_2 for c in v.contacts):
                n_leg += 1
    res["sanity_random"] = {"n": 200, "invalid": n_inv, "with_leg_proxy_contact": n_leg}
    ok = all(r["invalid_waypoints"] == 0 and not r["leg_contacts"] for k, r in res.items() if k != "sanity_random") and n_leg > 0
    msg = f"sanity: {n_inv}/200 random poses invalid, {n_leg} touch a leg proxy; " + "; ".join(f"{k}: SUCCESS {v['planning_time_s']:.2f} s plan ({v['wall_s']:.1f} s wall), {v['waypoints']} pts, "
                    f"{v['invalid_waypoints']} invalid, leg contacts {len(v['leg_contacts'])}" for k, v in res.items() if k != "sanity_random")
    done(ok, msg, out, res)


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
