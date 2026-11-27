"""RclpyArmPort: the real ArmPort (rclpy). NOT run yet (written 2026-11-27; same interfaces as scripts/grasp_test.py).

  /arm_controller/follow_joint_trajectory   control_msgs/FollowJointTrajectory (direct joint goals / waypoints)
  /move_action                              moveit_msgs/MoveGroup (plan + execute joint goals) when move_group runs
  /gripper_controller/commands              std_msgs/Float64MultiArray (effort per finger, + = open)
  /joint_states                             arm + finger positions
  /wrist_camera/{depth_image,camera_info}   + /wrist_camera/trigger (std_msgs/Bool, triggered camera)
  /speed_limit                              nav2_msgs/SpeedLimit (controller_server; absolute m/s, 0 = no limit)
  /camera/{depth_image,camera_info}         body depth camera: fitted floor plane -> MoveIt's scene (scripts/moveit_floor.py)
  /get_planning_scene, /apply_planning_scene  floor + the grasp target as collision objects (as scripts/grasp_test.py)
Own node + executor thread (the body port, RclpyPort, has its own). Blocking calls wait on futures with wall timeouts.
"""
from __future__ import annotations

import threading
import time

import numpy as np


class RclpyArmPort:
    def __init__(self, node_name: str = "gurdy_fetch_arm", moveit_wait_s: float = 5.0):
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from sensor_msgs.msg import CameraInfo, Image, JointState
        from std_msgs.msg import Bool, Float64MultiArray
        from gurdy_core import arm as A
        if not rclpy.ok():
            rclpy.init()
        self.A, self.ARM = A, A.load_arm()
        self.n = Node(node_name)
        self.FJT, self.F64, self.Bool = FollowJointTrajectory, Float64MultiArray, Bool
        self.js, self.depth, self.K, self._lock = {}, None, None, threading.Lock()
        self.n.create_subscription(JointState, "/joint_states", self._js, 20)
        self.n.create_subscription(Image, "/wrist_camera/depth_image", self._depth, 2)
        self.n.create_subscription(CameraInfo, "/wrist_camera/camera_info", lambda m: setattr(self, "K", (m.k[0], m.k[4], m.k[2], m.k[5])), 2)
        self.body_depth, self.body_info = None, None
        self.n.create_subscription(Image, "/camera/depth_image", lambda m: setattr(self, "body_depth", m), 2)
        self.n.create_subscription(CameraInfo, "/camera/camera_info", lambda m: setattr(self, "body_info", m), 2)
        self.trig = self.n.create_publisher(Bool, "/wrist_camera/trigger", 2)
        self.grip = self.n.create_publisher(Float64MultiArray, "/gripper_controller/commands", 2)
        try:
            from nav2_msgs.msg import SpeedLimit
            self.SpeedLimit = SpeedLimit
            self.limit_pub = self.n.create_publisher(SpeedLimit, "/speed_limit", 2)
        except ImportError:                                              # nav2_msgs missing: no limit (logged by the caller)
            self.SpeedLimit, self.limit_pub = None, None
        self.arm_ac = ActionClient(self.n, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.mg = None
        try:
            from moveit_msgs.action import MoveGroup
            self.MoveGroup = MoveGroup
            self.mg = ActionClient(self.n, MoveGroup, "/move_action")
        except ImportError:
            pass
        self._ex = MultiThreadedExecutor(num_threads=2)
        self._ex.add_node(self.n)
        threading.Thread(target=self._ex.spin, daemon=True).start()
        self._mg_ok = bool(self.mg and self.mg.wait_for_server(timeout_sec=moveit_wait_s))

    # ---- callbacks
    def _js(self, m):
        with self._lock:
            self.js.update(dict(zip(m.name, m.position)))

    def _depth(self, m):
        self.depth = (time.monotonic(), np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width).copy())

    @staticmethod
    def _done(fut, timeout):
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    # ---- ArmPort
    def arm_ready(self):
        return self.arm_ac.wait_for_server(timeout_sec=2.0)

    def moveit_available(self):
        return self._mg_ok

    def arm_q(self):
        with self._lock:
            if not all(n in self.js for n in self.A.ARM_JOINTS):
                return None
            return [float(self.js[n]) for n in self.A.ARM_JOINTS]

    def _direct(self, qs, seconds):
        from trajectory_msgs.msg import JointTrajectoryPoint
        g = self.FJT.Goal()
        g.trajectory.joint_names = list(self.A.ARM_JOINTS)
        for i, q in enumerate(qs):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            t = seconds * (i + 1) / len(qs)
            pt.time_from_start.sec, pt.time_from_start.nanosec = int(t), int((t % 1) * 1e9)
            g.trajectory.points.append(pt)
        h = self._done(self.arm_ac.send_goal_async(g), 10.0)
        if h is None or not h.accepted:
            return False
        r = self._done(h.get_result_async(), seconds + 10.0)
        return r is not None and r.result.error_code == 0

    def _moveit(self, q):
        from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
        goal = self.MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        c = Constraints()
        for n, v in zip(self.A.ARM_JOINTS, q):
            c.joint_constraints.append(JointConstraint(joint_name=n, position=float(v), tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        goal.request.goal_constraints.append(c)
        goal.planning_options.plan_only = False
        h = self._done(self.mg.send_goal_async(goal), 10.0)
        if h is None or not h.accepted:
            return False
        r = self._done(h.get_result_async(), 40.0)
        return r is not None and r.result.error_code.val == MoveItErrorCodes.SUCCESS

    def move_arm(self, qs, seconds, use_moveit=False):
        if use_moveit and self._mg_ok and len(qs) == 1:
            return self._moveit(qs[0])
        return self._direct(qs, seconds)

    def gripper(self, effort):
        self.grip.publish(self.F64(data=[float(effort), float(effort)]))

    def fingers(self):
        with self._lock:
            l, r = self.js.get("arm_finger_left_joint"), self.js.get("arm_finger_right_joint")
        return None if l is None or r is None else (float(l), float(r))

    def wrist_view(self):
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            self.trig.publish(self.Bool(data=True))
            time.sleep(0.15)
            d = self.depth
            q = self.arm_q()
            if d and d[0] > t0 and self.K and q is not None:
                return d[1], self.K, self.A.T_base_wrist_optical(self.ARM, q)
        return None

    def settle(self, q, tol=0.003, timeout=3.0):
        """Wait until the effort-controlled arm is within `tol` rad of q (integral term) or `timeout`; -> max error (rad)."""
        t0, err = time.monotonic(), float("inf")
        while time.monotonic() - t0 < timeout:
            m = self.arm_q()
            if m is not None:
                err = float(np.max(np.abs(np.asarray(m) - np.asarray(q, float))))
                if err < tol:
                    break
            time.sleep(0.1)
        return err

    def _scene_clients(self):
        if not hasattr(self, "_aps"):
            from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
            self.APS, self.GPS = ApplyPlanningScene, GetPlanningScene
            self._aps = self.n.create_client(ApplyPlanningScene, "/apply_planning_scene")
            self._gps = self.n.create_client(GetPlanningScene, "/get_planning_scene")
        return self._aps.wait_for_service(timeout_sec=5.0) and self._gps.wait_for_service(timeout_sec=5.0)

    def add_floor(self, margin=0.005, timeout=10.0) -> dict:
        """Fitted floor plane (body depth camera) -> `floor` box in MoveIt's scene, floor-vs-leg allowed (scripts/moveit_floor.py)."""
        import sys
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
        from moveit_floor import ACM_REQUEST, fit_floor_from, floor_scene
        from moveit_msgs.msg import PlanningSceneComponents
        t0 = time.monotonic()
        while (self.body_depth is None or self.body_info is None) and time.monotonic() - t0 < timeout:
            time.sleep(0.1)
        fl = fit_floor_from(self.body_depth, self.body_info)
        if not self._mg_ok or not self._scene_clients():
            return {**fl, "added": False, "why": "move_group scene services not available"}
        r = self._done(self._gps.call_async(self.GPS.Request(components=PlanningSceneComponents(components=ACM_REQUEST))), 10.0)
        if r is None:
            return {**fl, "added": False, "why": "get_planning_scene timed out"}
        scene, _ = floor_scene(fl, r.scene.allowed_collision_matrix, margin)
        r = self._done(self._aps.call_async(self.APS.Request(scene=scene)), 10.0)
        return {**fl, "added": bool(r is not None and r.success)}

    def target_in_scene(self, cls, xy_base, on=True, pad=0.01) -> bool:
        """Add / remove the grasp target (class-size box / cylinder + pad, on the floor, base_link) for the big MoveIt moves:
        otherwise the arm swing can knock it away (grasp_test fixed5x5; fetch dry run 2026-11-27: can pushed 6 cm)."""
        if not self._mg_ok or not self._scene_clients():
            return False
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
        from shape_msgs.msg import SolidPrimitive
        o = self.A.PICKABLE[cls]
        co = CollisionObject(id="grasp_target", operation=CollisionObject.ADD if on else CollisionObject.REMOVE)
        co.header.frame_id = "base_link"
        if on:
            h = (o["size"][2] if o["shape"] == "box" else o["height"]) + pad
            prim = (SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[o["size"][0] + 2 * pad, o["size"][1] + 2 * pad, h])
                    if o["shape"] == "box" else SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[h, o["radius"] + pad]))
            p = Pose()
            p.position.x, p.position.y, p.position.z = float(xy_base[0]), float(xy_base[1]), self.A.ground_z(self.A.load_robot()) + h / 2
            p.orientation.w = 1.0
            co.primitives, co.primitive_poses = [prim], [p]
        r = self._done(self._aps.call_async(self.APS.Request(scene=PlanningScene(
            is_diff=True, world=PlanningSceneWorld(collision_objects=[co])))), 5.0)
        return bool(r is not None and r.success)

    def set_speed_limit(self, mps):
        if self.limit_pub is None:
            return
        m = self.SpeedLimit()
        m.percentage, m.speed_limit = False, float(mps or 0.0)
        self.limit_pub.publish(m)

    def wait(self, seconds):
        time.sleep(seconds)
