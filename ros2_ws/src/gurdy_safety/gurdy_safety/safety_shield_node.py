"""LiDAR safety shield (control barrier function): /cmd_vel -> /cmd_vel_shielded.

    agent / teleop / Nav2 (collision_monitor) -> /cmd_vel -> safety_shield_node -> /cmd_vel_shielded
        -> [estop_gate_node -> /cmd_vel_safe] -> gait_node / policy_node

Minimally changes each command so no LiDAR point gets closer than `d_safe` to the robot footprint (robot.yaml, the Nav2
footprint), whatever produced it. Logic and maths: gurdy_core.safety_shield (unit-tested, + 2D simulator scenarios).
The E-STOP gate stays AFTER the shield, so a latched stop beats everything; the shield never creates motion from a zero
command.

Timing: wall clock (time.monotonic), like estop_gate_node. Commands are re-filtered at `rate_hz` with the newest scan while
the last input is younger than `cmd_timeout` (a sparse producer still gets stopped by an obstacle that appears later);
after that nothing is published and the controller's own command timeout applies.
Fail safe: no scan yet, or no scan for `scan_timeout` s -> zero command.
Status: /shield/status (std_msgs/String, JSON: state pass|active|stale|no_scan, active, reduction 0..1, min_clearance m,
n_constraints, n_self_hits, scan_age s, cmd_in, cmd_out) at 5 Hz while commands flow and on every state change.
Start it with `sim.launch.py shield:=true` (default false).
"""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from gurdy_core.robot import load_robot
from gurdy_core.safety_shield import SafetyShield, ShieldConfig
from gurdy_navigation.footprint import footprint_polygon


class SafetyShieldNode(Node):
    def __init__(self):
        super().__init__("safety_shield_node")
        p = {"robot_yaml": "", "input_topic": "/cmd_vel", "output_topic": "/cmd_vel_shielded", "scan_topic": "/scan",
             "status_topic": "/shield/status", "d_safe": 0.10, "alpha": 2.0, "self_margin": 0.02, "influence": 0.60,
             "footprint_margin": 0.04, "scan_timeout": 0.5, "cmd_timeout": 0.5, "rate_hz": 20.0, "scale_only": False}
        for k, v in p.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value          # noqa: E731
        robot = load_robot(g("robot_yaml") or None)
        lim = robot.command_limits
        cfg = ShieldConfig(footprint=footprint_polygon(robot, float(g("footprint_margin"))),
                           d_safe=float(g("d_safe")), alpha=float(g("alpha")),
                           limits=(float(lim["vx"]), float(lim["vy"]), float(lim["wz"])),
                           self_margin=float(g("self_margin")), influence=float(g("influence")),
                           scan_timeout=float(g("scan_timeout")), scale_only=bool(g("scale_only")),
                           lidar_origin=tuple(robot.sensors["lidar"]["pos"][:2]))
        self.shield = SafetyShield(cfg)
        self.cmd_timeout = float(g("cmd_timeout"))
        self.cmd, self.cmd_t = None, None
        self.last_state, self.last_status_t = None, 0.0
        self.pub = self.create_publisher(Twist, g("output_topic"), 10)
        self.status_pub = self.create_publisher(String, g("status_topic"), 10)
        self.create_subscription(Twist, g("input_topic"), self.on_cmd, 10)
        self.create_subscription(LaserScan, g("scan_topic"), self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0 / float(g("rate_hz")), self.tick)
        self.get_logger().info(f"shield: {g('input_topic')} -> {g('output_topic')}, d_safe {cfg.d_safe} m, "
                               f"alpha {cfg.alpha} 1/s, scan {g('scan_topic')} (timeout {cfg.scan_timeout} s)")

    @staticmethod
    def now():
        return time.monotonic()

    def on_scan(self, m: LaserScan):
        self.shield.update_laserscan(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max, self.now())

    def on_cmd(self, m: Twist):
        self.cmd, self.cmd_t = (m.linear.x, m.linear.y, m.angular.z), self.now()
        self.publish_filtered()

    def tick(self):
        if self.cmd_t is not None and self.now() - self.cmd_t <= self.cmd_timeout:
            self.publish_filtered()

    def publish_filtered(self):
        t = self.now()
        u, st = self.shield.filter(self.cmd, t)
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = (float(v) for v in u)
        self.pub.publish(out)
        if st.state != self.last_state:
            if st.state in ("stale", "no_scan"):
                self.get_logger().warn(f"shield: {st.state} -> zero velocity")
            elif self.last_state in ("stale", "no_scan"):
                self.get_logger().info("shield: scan back")
        if st.state != self.last_state or t - self.last_status_t >= 0.2:
            d = st.as_dict()
            d["cmd_in"] = [round(v, 4) for v in self.cmd]
            d["cmd_out"] = [round(v, 4) for v in u]
            self.status_pub.publish(String(data=json.dumps(d)))
            self.last_state, self.last_status_t = st.state, t


def main():
    rclpy.init()
    n = SafetyShieldNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
