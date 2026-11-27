#!/usr/bin/env python3
"""Phase 9 first grasp test in Gazebo lab4_fetch, ROS side (NOT RUN YET). Started by scripts/grasp_test.sh inside the sim
container with the stack up (arm:=true world:=lab4_fetch controller:=tripod, optionally move_group).

    python3 scripts/grasp_test.py OUT_DIR [--classes small_box can ...] [--tries 3] [--step teleport|walk] [--yaw wrist|gt|none]
                                          [--moveit auto|off] [--grip-effort 3.0]

Per try (5 classes x 3 tries by default; object = the lab4_fetch instance of the class, gurdy_core.grasp_test):
  0. reset: gripper open, arm `folded`, object set back to its ground-truth pose, robot TELEPORTED (gz set_pose) so the
     object lies at the class grasp zone + a known offset (TRY_OFFSETS) behind it; settle 3 s sim
  1. arm -> view pose (servo_view / servo_view_far), wrist camera triggered
  2. visual servo (gurdy_core.visual_servo.VisualServo, wrist depth + fitted floor + object3d): body steps are
     `teleport` (default: exact steps, isolates the grasp from walking) or `walk` (closed loop /cmd_vel 0.05 m/s on GT odom)
  3. grasp plan at the servo's estimate (gurdy_core.grasp_test.grasp_plan). Wrist roll from the object yaw:
     `--yaw wrist` (DEFAULT): gurdy_core.wrist_yaw.estimate_yaw on a fresh wrist frame (cube: min-area rectangle of the top
     face; mini toolbox: handle direction; symmetric classes: none); below `--yaw-min-conf` the roll falls back to 0.
     `--yaw gt`: ground truth (comparison only, an oracle). `--yaw none`: roll 0. Every try records the wrist estimate
     and its error vs ground truth whatever the mode.
  4. pre-grasp (MoveIt if /move_action answers, else a direct joint goal) -> grasp (direct, slow straight line)
  5. close: CONTACT physics only (effort on both finger joints, `--grip-effort` N each; no DetachableJoint / attach)
  6. lift 5 cm (5 IK waypoints, direct), hold 5 s; object pose sampled from /world/lab4_fetch/dynamic_pose/info
  7. classify (success / slipped / dropped / missed, + knocked flag), put it back down, open, arm folded
Writes OUT_DIR/tries.jsonl (one row per try, phase times, servo log, finger positions, z samples) and OUT_DIR/summary.json,
and prints one line per class: 'PASS grasp_<class> ...' or 'FAIL grasp_<class> ...' (PASS: >= 2 of 3 successes, no drop).

Unverified assumptions (check on the first run, docs/results/phase9g_grasp_test.md): finger effort sign (+ = open), the
protobuf text format of dynamic_pose/info (zero fields omitted), set_pose on the robot keeps it standing, the wrist
camera trigger, contact grasp stability in DART at 1 ms.
"""
import argparse
import json
import math
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "gurdy_core"))
sys.path.insert(0, str(ROOT / "scripts"))
import gen_world_lab4 as L4  # noqa: E402
from gurdy_core import arm as A  # noqa: E402
from gurdy_core import grasp_test as G  # noqa: E402
from gurdy_core.visual_servo import VIEW_POSE, ServoParams, VisualServo  # noqa: E402
from gurdy_core import wrist_yaw as WY  # noqa: E402

WORLD, ROBOT_MODEL = "lab4_fetch", "gurdy"
BASE_Z = 0.115
GT_FILE = ROOT / "docs/results/data/lab4_fetch/lab4_fetch_gt.json"
ARM, RB = A.load_arm(), A.load_robot()


# ------------------------------------------------------------------ gz helpers
def gz(args, timeout=10.0):
    try:
        r = subprocess.run(["gz", *args], capture_output=True, text=True, timeout=timeout)
        return r.returncode == 0, r.stdout + r.stderr
    except Exception as e:                                          # noqa: BLE001
        return False, repr(e)


def set_pose(name, x, y, z, yaw):
    req = (f'name: "{name}" position {{x: {x:.4f} y: {y:.4f} z: {z:.4f}}} '
           f'orientation {{x: 0 y: 0 z: {math.sin(yaw / 2):.6f} w: {math.cos(yaw / 2):.6f}}}')
    ok, out = gz(["service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                  "--timeout", "3000", "--req", req])
    return ok and "data: true" in out


class PoseStream:
    """Latest pose per model from `gz topic -e -t /world/W/dynamic_pose/info` (protobuf text; zero fields are omitted, so a
    missing x / y / z means 0). Thread + subprocess; `get(name)` -> (x, y, z, wall_time) or None."""

    def __init__(self, topic):
        self.latest, self.lock = {}, threading.Lock()
        self.p = subprocess.Popen(["gz", "topic", "-e", "-t", topic], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                  text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        parser = G.PoseTextParser()
        for line in self.p.stdout:
            r = parser.feed(line)
            if r:
                with self.lock:
                    self.latest[r[0]] = (r[1], r[2], r[3], time.time())

    def get(self, name, max_age=1.0):
        with self.lock:
            v = self.latest.get(name)
        return v if v and time.time() - v[3] <= max_age else None

    def close(self):
        self.p.kill()


# ------------------------------------------------------------------ ROS node
class Rig(Node):
    def __init__(self, a):
        super().__init__("grasp_test")
        from control_msgs.action import FollowJointTrajectory
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import CameraInfo, Image, JointState
        from std_msgs.msg import Bool, Float64MultiArray
        self.FJT, self.Twist, self.F64, self.Bool = FollowJointTrajectory, Twist, Float64MultiArray, Bool
        self.a = a
        self.js, self.odom, self.depth, self.K = {}, None, None, None
        self.create_subscription(JointState, "/joint_states", lambda m: self.js.update(dict(zip(m.name, m.position))), 20)
        self.create_subscription(Odometry, "/odom/ground_truth", self._odom, 20)
        self.create_subscription(Image, "/wrist_camera/depth_image", self._depth, 2)
        self.create_subscription(CameraInfo, "/wrist_camera/camera_info", lambda m: setattr(self, "K", (m.k[0], m.k[4], m.k[2], m.k[5])), 2)
        self.trig = self.create_publisher(Bool, "/wrist_camera/trigger", 2)
        self.grip = self.create_publisher(Float64MultiArray, "/gripper_controller/commands", 2)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 2)
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.mg = None
        if a.moveit != "off":
            from moveit_msgs.action import MoveGroup
            self.MoveGroup = MoveGroup
            ac = ActionClient(self, MoveGroup, "/move_action")
            self.mg = ac if ac.wait_for_server(timeout_sec=15.0) else None
        self.poses = PoseStream(f"/world/{WORLD}/dynamic_pose/info")

    def _odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, yaw)

    def _depth(self, m):
        self.depth = (time.monotonic(), np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width).copy())

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def now(self):
        return self.odom[0] if self.odom else time.monotonic()

    def q_arm(self):
        return np.array([self.js.get(n, 0.0) for n in A.ARM_JOINTS])

    def fingers(self):
        return [round(self.js.get("arm_finger_left_joint", float("nan")), 4), round(self.js.get("arm_finger_right_joint", float("nan")), 4)]

    # ---- arm
    def move_direct(self, qs, seconds):
        from trajectory_msgs.msg import JointTrajectoryPoint
        g = self.FJT.Goal()
        g.trajectory.joint_names = A.ARM_JOINTS
        qs = [qs] if np.ndim(qs[0]) == 0 else qs
        for i, q in enumerate(qs):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            t = seconds * (i + 1) / len(qs)
            pt.time_from_start.sec, pt.time_from_start.nanosec = int(t), int((t % 1) * 1e9)
            g.trajectory.points.append(pt)
        f = self.arm.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        h = f.result()
        if h is None or not h.accepted:
            return False
        r = h.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=seconds + 10.0)
        return r.result() is not None and r.result().result.error_code == 0

    def move_moveit(self, q):
        from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
        goal = self.MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        c = Constraints()
        for n, v in zip(A.ARM_JOINTS, q):
            c.joint_constraints.append(JointConstraint(joint_name=n, position=float(v), tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        goal.request.goal_constraints.append(c)
        goal.planning_options.plan_only = False
        f = self.mg.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        h = f.result()
        if h is None or not h.accepted:
            return False
        r = h.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=40.0)
        return r.result() is not None and r.result().result.error_code.val == MoveItErrorCodes.SUCCESS

    def move(self, q, seconds=3.0, log=None):
        """Big motion: MoveIt when available (falls back to a direct goal on failure), else direct."""
        if self.mg is not None:
            if self.move_moveit(q):
                (log or []).append("moveit")
                return True
            (log or []).append("moveit_failed->direct")
        (log or []).append("direct")
        return self.move_direct(q, seconds)

    def gripper(self, effort):
        self.grip.publish(self.F64(data=[float(effort), float(effort)]))

    # ---- body (visual servo interface)
    def wrist_view(self):
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            self.trig.publish(self.Bool(data=True))
            self.spin_for(0.15)
            if self.depth and self.depth[0] > t0 and self.K:
                return self.depth[1], self.K, A.T_base_wrist_optical(ARM, self.q_arm())
        raise RuntimeError("no wrist depth frame after the trigger")

    def step(self, dx, dy):
        t, x, y, _, yaw = self.odom
        c, s = math.cos(yaw), math.sin(yaw)
        tx, ty = x + c * dx - s * dy, y + s * dx + c * dy
        if self.a.step == "teleport":
            set_pose(ROBOT_MODEL, tx, ty, BASE_Z, yaw)
            self.spin_for(2.0)
            return
        t_end = time.monotonic() + 6.0                         # walk: closed loop on GT odom, 0.05 m/s, yaw held
        while time.monotonic() < t_end:
            _, x, y, _, yaw = self.odom
            ex, ey = tx - x, ty - y
            if math.hypot(ex, ey) < 0.005:
                break
            c, s = math.cos(yaw), math.sin(yaw)
            bx, by = c * ex + s * ey, -s * ex + c * ey
            n = max(math.hypot(bx, by), 1e-6)
            v = min(0.05, n * 2.0)
            m = self.Twist()
            m.linear.x, m.linear.y = bx / n * v, by / n * v
            self.cmd.publish(m)
            self.spin_for(0.05)
        self.cmd.publish(self.Twist())
        self.spin_for(1.5)


# ------------------------------------------------------------------ one try
def one_try(rig: Rig, obj: dict, k: int, a) -> dict:
    cls, name = obj["class"], obj["model"]
    off = G.TRY_OFFSETS[k % len(G.TRY_OFFSETS)]
    rec = {"class": cls, "object": obj["name"], "try": k, "offset": off, "log": [], "phase_s": {}}
    tt = lambda: round(rig.now(), 3)                                # noqa: E731
    # 0. reset
    rig.gripper(+abs(a.grip_effort))
    rig.move_direct(ARM.poses["folded"], 3.0)
    set_pose(name, obj["x"], obj["y"], 0.0005, obj.get("yaw", 0.0))
    rig.spin_for(1.0)
    walls = L4.walls()
    pl = G.place_robot(obj, walls, G.other_obstacles(GT, obj["name"]), off)
    if pl is None:
        return {**rec, "outcome": "error", "detail": "no placement"}
    rec["placement"] = pl
    if not set_pose(ROBOT_MODEL, pl["x"], pl["y"], BASE_Z, pl["yaw"]):
        return {**rec, "outcome": "error", "detail": "robot set_pose failed"}
    rig.spin_for(3.0)
    if rig.odom is None or rig.odom[3] < 0.06:
        return {**rec, "outcome": "error", "detail": f"robot not standing after the teleport (z {rig.odom and rig.odom[3]})"}
    rest = rig.poses.get(name, 3.0)
    if rest is None:
        return {**rec, "outcome": "error", "detail": f"no pose of {name} in dynamic_pose/info"}
    t0 = tt()
    # 1. view pose + 2. servo
    rig.move(ARM.poses[VIEW_POSE[cls]], 3.0, rec["log"])
    rig.spin_for(1.0)
    _, x, y, _, yaw = rig.odom
    prior = G.world_to_base((x, y, yaw), (obj["x"], obj["y"]))
    arm_ok = lambda c, xy: G.grasp_plan(ARM, RB, c, xy).ok          # noqa: E731
    try:
        sres = VisualServo(rig, ServoParams(), arm_reachable=arm_ok).run(cls, prior_xy=prior)
    except Exception as e:                                          # noqa: BLE001
        return {**rec, "outcome": "error", "detail": f"servo: {e!r}"}
    rec["servo"] = {"ok": sres.ok, "code": sres.code, "steps": sres.steps, "residual": sres.residual, "log": sres.log}
    rec["phase_s"]["servo"] = round(tt() - t0, 2)
    _, x, y, _, yaw = rig.odom
    rec["true_xy_base"] = [round(v, 4) for v in G.world_to_base((x, y, yaw), rest[:2])]
    if not sres.ok:
        return {**rec, "outcome": "missed", "detail": f"servo {sres.code}", "time_s": None}
    est = np.array(sres.obs.xy)
    rec["est_xy_base"] = [round(float(v), 4) for v in est]
    # 3. yaw (wrist estimate always recorded and compared with ground truth) + plan
    gt_b = math.atan2(math.sin(obj.get("yaw", 0.0) - yaw), math.cos(obj.get("yaw", 0.0) - yaw))
    try:
        dpt, Kw, Tw = rig.wrist_view()
        ye = WY.estimate_yaw(dpt, Kw, Tw, cls, xy_hint=est)
    except Exception as e:                                          # noqa: BLE001
        ye = WY.YawEstimate(None, 0.0, "none", WY.PERIOD.get(cls), reason=f"wrist view: {e!r}")
    rec["yaw"] = G.yaw_record(cls, ye, gt_b, a.yaw, a.yaw_min_conf)
    yaw_b = rec["yaw"]["used"]
    plan = G.grasp_plan(ARM, RB, cls, est, yaw_b)
    rec["plan"] = plan.to_dict()
    if not plan.ok:
        return {**rec, "outcome": "missed", "detail": f"plan: {plan.reason}", "time_s": None}
    # 4. approach
    t1 = tt()
    rig.move(plan.pre, 3.0, rec["log"])
    rig.move_direct(plan.grasp, 2.0)
    before = rig.poses.get(name)
    rec["phase_s"]["approach"] = round(tt() - t1, 2)
    # 5. close (contact only)
    rig.gripper(-abs(a.grip_effort))
    rig.spin_for(1.5)
    rec["fingers_closed"] = rig.fingers()
    # 6. lift + hold
    t2 = tt()
    lift_z, hold_z = [], []
    done = threading.Event()

    def sampler(dst):
        while not done.is_set():
            p = rig.poses.get(name)
            if p:
                dst.append(p[2])
            time.sleep(0.05)
    th = threading.Thread(target=sampler, args=(lift_z,), daemon=True)
    th.start()
    rig.move_direct(plan.lift, 2.5)
    done.set()
    th.join()
    done.clear()
    th = threading.Thread(target=sampler, args=(hold_z,), daemon=True)
    th.start()
    t_hold = rig.now()
    while rig.now() - t_hold < G.HOLD_S:
        rig.spin_for(0.1)
    done.set()
    th.join()
    rec["phase_s"]["lift_hold"] = round(tt() - t2, 2)
    rec["fingers_hold"] = rig.fingers()
    res = G.classify(rest[2], lift_z, hold_z, rest[:2], before[:2] if before else None)
    rec.update(res.to_dict())
    rec["time_s"] = round(tt() - t0, 2)
    rec["z_samples"] = {"rest": rest[2], "lift": [round(z, 4) for z in lift_z[::2]], "hold": [round(z, 4) for z in hold_z[::4]]}
    # 7. put it back down, open, fold
    rig.move_direct(plan.grasp, 2.0)
    rig.gripper(+abs(a.grip_effort))
    rig.spin_for(1.0)
    rig.move_direct(plan.pre, 1.5)
    rig.move(ARM.poses["folded"], 3.0, rec["log"])
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--classes", nargs="*", default=list(G.SMALL))
    ap.add_argument("--tries", type=int, default=3)
    ap.add_argument("--step", choices=("teleport", "walk"), default="teleport")
    ap.add_argument("--yaw", choices=("wrist", "gt", "none"), default="wrist", help="wrist estimate (default), ground truth (comparison), 0")
    ap.add_argument("--yaw-min-conf", type=float, default=0.5, help="below this wrist-yaw confidence the roll is 0")
    ap.add_argument("--moveit", choices=("auto", "off"), default="auto")
    ap.add_argument("--grip-effort", type=float, default=3.0, help="N per finger (joint effort limit 10 N)")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    rig = Rig(a)
    rows = []
    try:
        if not rig.arm.wait_for_server(timeout_sec=30.0):
            print("FAIL grasp_setup no /arm_controller/follow_joint_trajectory")
            return 1
        for _ in range(100):
            rig.spin_for(0.1)
            if rig.odom and rig.js:
                break
        print(f"grasp test: moveit {'yes' if rig.mg else 'no (direct joint goals)'}, step {a.step}, yaw {a.yaw}, grip {a.grip_effort} N", flush=True)
        with open(out / "tries.jsonl", "w") as fh:
            for cls in a.classes:
                obj = next(o for o in GT if o["class"] == cls)
                for k in range(a.tries):
                    t_wall = time.time()
                    try:
                        r = one_try(rig, obj, k, a)
                    except Exception as e:                          # noqa: BLE001
                        r = {"class": cls, "object": obj["name"], "try": k, "outcome": "error", "detail": repr(e)}
                    r["wall_s"] = round(time.time() - t_wall, 1)
                    rows.append(r)
                    fh.write(json.dumps(r, default=float) + "\n")
                    fh.flush()
                    y = r.get("yaw") or {}
                    print(f"  {cls} try {k + 1}: {r['outcome']} {r.get('detail', '')} (lifted {r.get('lifted')}, sank {r.get('sank')}, "
                          f"time {r.get('time_s')} s sim; yaw {y.get('method')} conf {y.get('confidence')} err {y.get('err_deg')} deg, "
                          f"used {y.get('used_src')})", flush=True)
        summ = G.summarize(rows)
        (out / "summary.json").write_text(json.dumps(summ, indent=1) + "\n")
        for cls in a.classes:
            s = summ.get(cls)
            if s is None:
                continue
            print(f"{'PASS' if s['pass'] else 'FAIL'} grasp_{cls} {s['success']}/{s['tries']} success, {s['slipped']} slipped, "
                  f"{s['dropped']} dropped, {s['missed']} missed, {s['error']} errors, {s['knocked']} knocked, "
                  f"mean time {s['mean_time_s']} s sim, wrist yaw err median {s['yaw_err_median_deg']} / max {s['yaw_err_max_deg']} deg "
                  f"({s['yaw_low_conf']} low confidence)", flush=True)
    finally:
        rig.poses.close()
        rig.destroy_node()
        rclpy.try_shutdown()
    return 0


GT = json.loads(GT_FILE.read_text())["objects"]

if __name__ == "__main__":
    sys.exit(main())
