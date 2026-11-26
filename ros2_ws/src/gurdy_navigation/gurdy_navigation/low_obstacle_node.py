"""low_obstacle_node (Phase 9c, NOT run in Gazebo yet; not in any launch file): body depth camera -> low obstacles.

    /camera/depth_image (32FC1) + /camera/camera_info + /joint_states (+ TF odom <- camera_optical_frame)
      -> gurdy_core.low_obstacles.extract (floor RANSAC per frame, self filter with leg capsules from /joint_states
         and, with arm:=true, the Phase 9a arm capsules from its joint states)
    out (all base_link, stamped with the depth image):
      /low_obstacles/marking   PointCloud2  obstacle points             -> Nav2 local costmap low_obstacle_layer (marking)
      /low_obstacles/clearing  PointCloud2  floor points (decimated)    -> same layer (clearing ray traces)
      /low_obstacles/shield    PointCloud2  current + remembered points -> safety_shield_node (use_depth_obstacles)
Memory: odom-frame points, ttl 10 s, forgotten when the camera sees through them (the camera sees only 60 deg ahead,
nothing closer than ~0.35 m in front of base_link). Without TF odom <- camera the memory is skipped (current frame only).
Rate limited to `max_rate_hz` (default 5) because the costmap updates at 5 Hz.

Phase 9f, wrist:=true (default false; needs arm:=true): the wrist RGB-D (/wrist_camera/depth_image + camera_info; it
renders only while picking, triggered) is a SECOND source with its own self filter: legs + arm + gripper (finger
opening from /joint_states) + the held object (/fetch/held_object, std_msgs/String JSON {"radius": r, "height": h}, ""
= nothing held). Its clouds go to /low_obstacles/wrist_marking + /low_obstacles/wrist_clearing (the costmap ray-traces
them from wrist_camera_optical_frame, see the overlay), and its points (if younger than wrist_timeout) are merged into
/low_obstacles/shield. So backing up to an object is not blind any more (the body camera looks forward only).
"""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from gurdy_core import low_obstacles as LO
from gurdy_core.robot import load_robot


def _quat_T(t, q):
    x, y, z, w = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return LO.make_T(R, t)


class LowObstacleNode(Node):
    def __init__(self):
        super().__init__("low_obstacle_node")
        P = self.declare_parameter
        P("robot_yaml", ""); P("arm", False); P("arm_yaml", ""); P("max_rate_hz", 5.0); P("max_range", 1.8)
        P("min_h", 0.025); P("max_h", 0.45); P("memory_ttl", 10.0); P("odom_frame", "odom")
        P("wrist", False); P("wrist_timeout", 0.5)
        g = lambda k: self.get_parameter(k).value          # noqa: E731
        self.robot = load_robot(g("robot_yaml") or None)
        self.T_bo = LO.T_base_optical(self.robot.sensors["depth_camera"])
        self.prm = LO.LowObstacleParams(max_range=float(g("max_range")), min_h=float(g("min_h")), max_h=float(g("max_h")),
                                        floor_prior_z=-float(self.robot.stance["body_height"]))
        self.arm = None
        if g("arm"):
            from gurdy_core import arm as A
            self.arm = A.load_arm(g("arm_yaml") or None)
        self.mem = LO.LowObstacleMemory(ttl=float(g("memory_ttl")))
        self.K, self.q, self.last = None, {}, 0.0
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.pub_mark = self.create_publisher(PointCloud2, "/low_obstacles/marking", 2)
        self.pub_clear = self.create_publisher(PointCloud2, "/low_obstacles/clearing", 2)
        self.pub_shield = self.create_publisher(PointCloud2, "/low_obstacles/shield", 2)
        self.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(Image, "/camera/depth_image", self.on_depth, qos_profile_sensor_data)
        self.wrist_res, self.wrist_t, self.K_w, self.held = None, None, None, None
        if g("wrist"):
            if self.arm is None:
                from gurdy_core import arm as A
                self.arm = A.load_arm(g("arm_yaml") or None)
            self.pub_wmark = self.create_publisher(PointCloud2, "/low_obstacles/wrist_marking", 2)
            self.pub_wclear = self.create_publisher(PointCloud2, "/low_obstacles/wrist_clearing", 2)
            self.create_subscription(CameraInfo, "/wrist_camera/camera_info", self.on_winfo, qos_profile_sensor_data)
            self.create_subscription(Image, "/wrist_camera/depth_image", self.on_wrist, qos_profile_sensor_data)
            self.create_subscription(String, "/fetch/held_object", self.on_held, 2)

    def on_info(self, m):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def on_winfo(self, m):
        self.K_w = (m.k[0], m.k[4], m.k[2], m.k[5])

    def on_held(self, m):
        try:
            d = json.loads(m.data) if m.data else None
            self.held = (float(d["radius"]), float(d.get("height", 0.0))) if d else None
        except (ValueError, KeyError, TypeError):
            self.get_logger().warn(f"bad /fetch/held_object {m.data!r}: treated as nothing held", throttle_duration_sec=10.0)
            self.held = None

    def on_wrist(self, m: Image):
        from gurdy_core import arm as A
        if self.K_w is None or m.encoding != "32FC1":
            return
        depth = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
        qa = [self.q.get(n, v) for n, v in zip(A.ARM_JOINTS, self.arm.poses["folded"])]
        opening = 2 * float(self.q.get("arm_finger_left_joint", 0.04))
        r = LO.extract_wrist(depth, self.K_w, self.robot, self.arm, qa, self.q, opening=max(opening, 0.02), held=self.held)
        self.wrist_res, self.wrist_t = r, time.monotonic()
        hdr = m.header
        hdr.frame_id = "base_link"
        self.pub_wmark.publish(point_cloud2.create_cloud_xyz32(hdr, r.obstacles.astype(np.float32).tolist()))
        self.pub_wclear.publish(point_cloud2.create_cloud_xyz32(hdr, r.clearing.astype(np.float32).tolist()))

    def on_js(self, m):
        self.q.update(dict(zip(m.name, m.position)))

    def self_capsules(self):
        caps = LO.leg_capsules(self.robot, self.q)
        if self.arm is not None:
            from gurdy_core import arm as A
            qa = [self.q.get(n, v) for n, v in zip(A.ARM_JOINTS, self.arm.poses["folded"])]
            caps += [(p0, p1, r) for _, p0, p1, r in A.arm_capsules(self.arm, qa)]
        return caps

    def on_depth(self, m: Image):
        now = time.monotonic()
        if self.K is None or now - self.last < 1.0 / float(self.get_parameter("max_rate_hz").value):
            return
        self.last = now
        if m.encoding != "32FC1":
            self.get_logger().warn(f"depth encoding {m.encoding} not supported", throttle_duration_sec=10.0)
            return
        depth = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
        r = LO.extract(depth, self.K, self.T_bo, self.prm, self.self_capsules(), tuple(np.array(self.robot.body_size) / 2))
        hdr = m.header
        hdr.frame_id = "base_link"
        self.pub_mark.publish(point_cloud2.create_cloud_xyz32(hdr, r.obstacles.astype(np.float32).tolist()))
        self.pub_clear.publish(point_cloud2.create_cloud_xyz32(hdr, r.clearing.astype(np.float32).tolist()))
        shield = r.obstacles
        if self.wrist_res is not None and time.monotonic() - self.wrist_t <= float(self.get_parameter("wrist_timeout").value):
            shield = np.vstack([shield, self.wrist_res.obstacles]) if len(self.wrist_res.obstacles) else shield
        try:
            tr = self.tf.lookup_transform(self.get_parameter("odom_frame").value, "base_link", m.header.stamp,
                                          timeout=rclpy.duration.Duration(seconds=0.05))
            T_ob = _quat_T([tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z],
                           [tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w])
            t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
            self.mem.clear_seen_through(depth, self.K, T_ob @ self.T_bo, self.prm)
            self.mem.add(LO.apply_T(T_ob, r.obstacles), t)
            self.mem.prune(t)
            mp = self.mem.points()
            if len(mp):
                shield = np.vstack([r.obstacles, LO.apply_T(np.linalg.inv(T_ob), mp)]) if len(r.obstacles) else \
                    LO.apply_T(np.linalg.inv(T_ob), mp)
        except Exception as e:                       # noqa: BLE001  (no odom TF yet: current frame only)
            self.get_logger().warn(f"no TF for the memory: {e}", throttle_duration_sec=10.0)
        self.pub_shield.publish(point_cloud2.create_cloud_xyz32(hdr, np.asarray(shield, np.float32).tolist()))
        if not r.plane_fitted:
            self.get_logger().warn("floor plane not fitted (too few floor points): prior plane used", throttle_duration_sec=10.0)


def main():
    rclpy.init()
    n = LowObstacleNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()
