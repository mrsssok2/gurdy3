"""costmap_recorder_node: save the global costmap + robot pose whenever planner_server logs a planning failure.

    ros2 run gurdy_navigation costmap_recorder_node --ros-args -p out_dir:=/path   (sim.launch.py costmap_record_dir:=/path)
Off unless started (launch arg empty by default). Listens to /rosout (planner_server failures, incl. the agent's own path
check, which also goes through planner_server), /global_costmap/costmap (+ _updates, Nav2 sends updates after the first
full map) and /odom/ground_truth. Logic and file format: gurdy_navigation.costmap_recorder.
"""
import time

import numpy as np
import rclpy
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from gurdy_navigation.costmap_recorder import parse_failure, save_snapshot


class CostmapRecorder(Node):
    def __init__(self):
        super().__init__("costmap_recorder_node")
        self.out_dir = self.declare_parameter("out_dir", "/tmp/costmap_failures").value
        topic = self.declare_parameter("costmap_topic", "/global_costmap/costmap").value
        self.min_gap = float(self.declare_parameter("min_interval_s", 0.5).value)   # one snapshot per burst of log lines
        self.grids, self.cur, self.pose, self.n, self.last_t = [], None, None, 0, 0.0
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, topic, self.on_grid, latched)
        self.create_subscription(OccupancyGridUpdate, topic + "_updates", self.on_update, 10)
        self.create_subscription(Odometry, "/odom/ground_truth", self.on_odom, 10)
        self.create_subscription(Log, "/rosout", self.on_log, 50)
        self.get_logger().info(f"costmap recorder: {topic} -> {self.out_dir}")

    def _push(self, stamp):
        g = dict(self.cur)
        g["data"] = self.cur["data"].copy()
        g["stamp"] = stamp
        self.grids = (self.grids + [g])[-2:]

    def on_grid(self, m: OccupancyGrid):
        i = m.info
        self.cur = {"data": np.asarray(m.data, dtype=np.int8).reshape(i.height, i.width),
                    "origin": (i.origin.position.x, i.origin.position.y), "res": i.resolution}
        self._push(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)

    def on_update(self, m: OccupancyGridUpdate):
        if self.cur is None:
            return
        d = self.cur["data"]
        patch = np.asarray(m.data, dtype=np.int8).reshape(m.height, m.width)
        d[m.y:m.y + m.height, m.x:m.x + m.width] = patch[:max(0, d.shape[0] - m.y), :max(0, d.shape[1] - m.x)]
        self._push(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)

    def on_odom(self, m: Odometry):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        import math
        self.pose = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_log(self, m: Log):
        f = parse_failure(m.name, m.msg)
        if f is None or not self.grids or time.monotonic() - self.last_t < self.min_gap:
            return
        self.last_t = time.monotonic()
        self.n += 1
        meta = save_snapshot(self.out_dir, self.n, f, self.grids, self.pose)
        self.get_logger().warn(f"planner failure #{self.n} recorded: {meta['files'][0]} "
                               f"(lethal cells near start: {[g.get('lethal_cells_within_0.3m_of_start') for g in meta['grids']]})")


def main():
    rclpy.init()
    n = CostmapRecorder()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
