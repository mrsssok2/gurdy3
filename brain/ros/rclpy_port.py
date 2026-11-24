"""RclpyPort: the real RosPort (rclpy node spun in a background thread). Only importable where ROS 2 is sourced
(the sim / ros image). NOT run against Gazebo yet - see docs/results/phase5_ros_backend.md for the test commands.

Topics / actions used (parameters can remap): TF map->base_link, /clock, /odometry/filtered (or /odom), /imu,
/policy/status, semantic_map/objects (JSON String, latched), /map, /scan, /gurdy/alerts (JSON String; interim until
the gurdy_interfaces/Event bus exists), navigate_to_pose + compute_path_to_pose (Nav2), cmd_vel_smoothed, /estop.
/battery (BatteryState, SIMULATED), /gurdy/leg_health (latched JSON), `explore` action (gurdy_interfaces/Explore; the explorer
must run with serve_action:=true). Until a battery / leg message arrives those return None (unknown, never invented).
"""
from __future__ import annotations

import json
import math
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from gurdy_interfaces.action import Explore
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.clock import Clock as RclpyClock, ClockType
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .port import NAV_ABORTED, NAV_CANCELED, NAV_OK, NAV_RUNNING

_STATUS = {4: NAV_OK, 5: NAV_CANCELED, 6: NAV_ABORTED, 1: NAV_RUNNING, 2: NAV_RUNNING, 3: NAV_RUNNING}   # action_msgs GoalStatus
_LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class RclpyPort:
    def __init__(self, map_id: str = "unknown", odom_topic: str = "/odometry/filtered", cmd_topic: str = "cmd_vel_smoothed",
                 node_name: str = "gurdy_agent_backend", spin: bool = True, interior_bounds=None):
        if not rclpy.ok():
            rclpy.init()
        self.node = n = Node(node_name, parameter_overrides=[])
        self._map_id = map_id
        self._bounds = interior_bounds
        self._lock = threading.Lock()
        self._t = None
        self._speed = None
        self._tilt = None
        self._policy = None
        self._objects = None
        self._map = None
        self._gcm = None
        self._lc: dict = {}
        self._scan = None
        self._alerts: deque = deque(maxlen=500)
        self._battery = None
        self._legs = None
        self._explores: dict[int, dict] = {}
        self._estop = False
        self._goals: dict[int, dict] = {}
        self._h = 0
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, n)
        n.create_subscription(ClockMsg, "/clock", self._on_clock, 10)
        n.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        n.create_subscription(Imu, "/imu", self._on_imu, qos_profile_sensor_data)
        n.create_subscription(String, "/policy/status", lambda m: setattr(self, "_policy", m.data), 10)
        n.create_subscription(String, "semantic_map/objects", self._on_objects, _LATCHED)
        n.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "_map", m), _LATCHED)
        n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: setattr(self, "_gcm", m), _LATCHED)
        n.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "_scan", m), qos_profile_sensor_data)
        n.create_subscription(String, "/gurdy/alerts", self._on_alert, 50)
        n.create_subscription(BatteryState, "/battery", lambda m: setattr(self, "_battery", 100.0 * m.percentage), 10)
        n.create_subscription(String, "/gurdy/leg_health", self._on_legs, _LATCHED)
        self._pub_cmd = n.create_publisher(Twist, cmd_topic, 10)
        self._zero_until = 0.0
        n.create_timer(0.05, self._zero_tick, clock=RclpyClock(clock_type=ClockType.STEADY_TIME))   # wall-clock timer: publishes only while a zero hold is active
        self._pub_estop = n.create_publisher(Bool, "/estop", _LATCHED)
        self._nav = ActionClient(n, NavigateToPose, "navigate_to_pose")
        self._plan = ActionClient(n, ComputePathToPose, "compute_path_to_pose")
        self._explore = ActionClient(n, Explore, "explore")
        if spin:
            self._ex = MultiThreadedExecutor(num_threads=3)
            self._ex.add_node(n)
            threading.Thread(target=self._ex.spin, daemon=True).start()

    # ---- callbacks
    def _on_clock(self, m):
        self._t = m.clock.sec + m.clock.nanosec * 1e-9

    def _on_odom(self, m):
        v = m.twist.twist.linear
        self._speed = math.hypot(v.x, v.y)

    def _on_imu(self, m):
        q = m.orientation
        gx, gy = 2 * (q.x * q.z - q.w * q.y), 2 * (q.w * q.x + q.y * q.z)
        gz = 1 - 2 * (q.x * q.x + q.y * q.y)
        self._tilt = math.degrees(math.acos(max(-1.0, min(1.0, gz))))

    def _on_objects(self, m):
        try:
            self._objects = json.loads(m.data).get("objects", [])
        except (ValueError, AttributeError):
            pass

    def _on_alert(self, m):
        try:
            a = json.loads(m.data)
            if isinstance(a, dict):
                self._alerts.append(a)
        except ValueError:
            pass

    def _on_legs(self, m):
        try:
            legs = json.loads(m.data).get("legs")
            if isinstance(legs, dict):
                self._legs = {str(k): str(v) for k, v in legs.items()}
        except ValueError:
            pass

    def _wait(self, fut, timeout: float):
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        return fut.result() if ev.wait(timeout) else None

    # ---- facts
    def now(self) -> float:
        return self._t if self._t is not None else 0.0

    def pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_link", rclpy.time.Time())
        except TransformException:
            return None
        q, p = t.transform.rotation, t.transform.translation
        return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def speed(self):
        return self._speed

    def tilt_deg(self):
        return self._tilt

    def policy_status(self):
        return self._policy

    def semantic_objects(self):
        return self._objects

    def map_id(self) -> str:
        return self._map_id

    def map_cell(self, x, y):
        m = self._map
        if m is None:
            return None
        i, j = int((x - m.info.origin.position.x) / m.info.resolution), int((y - m.info.origin.position.y) / m.info.resolution)
        if not (0 <= i < m.info.width and 0 <= j < m.info.height):
            return "unknown"
        v = m.data[j * m.info.width + i]
        return "unknown" if v < 0 else ("occupied" if v >= 65 else "free")

    def occupancy(self):
        """The current /map as (width, height, resolution, origin_x, origin_y, data) or None (for the dashboard)."""
        m = self._map
        if m is None:
            return None
        return m.info.width, m.info.height, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y, m.data

    def mapped_fraction(self):
        """Known share of the interior rectangle `interior_bounds` (xmin, xmax, ymin, ymax); None without bounds."""
        m = self._map
        if m is None or self._bounds is None:
            return None
        import numpy as np
        from gurdy_core import frontier as fr
        grid = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
        org = (m.info.origin.position.x, m.info.origin.position.y)
        return fr.map_stats(grid, m.info.resolution, org, self._bounds)["interior_fraction"]

    def obstacle_sectors(self, radius_m):
        s = self._scan
        if s is None:
            return []
        best: dict[int, float] = {}
        for k, r in enumerate(s.ranges):
            if not (s.range_min < r < min(radius_m, s.range_max)):
                continue
            b = math.degrees(s.angle_min + k * s.angle_increment)
            sec = int(((b + 22.5) % 360) // 45)
            best[sec] = min(best.get(sec, 1e9), r)
        return [{"bearing_deg": sec * 45.0 if sec < 5 else sec * 45.0 - 360.0, "distance_m": d} for sec, d in best.items()]

    def battery_pct(self):
        return self._battery             # SIMULATED (battery_sim_node)

    def leg_states(self):
        return self._legs

    def drain_alerts(self):
        out = []
        while self._alerts:
            out.append(self._alerts.popleft())
        return out

    # ---- Nav2
    def nav_ready(self) -> bool:
        return self._nav.server_is_ready()

    def global_costmap_ready(self) -> bool:
        m = self._gcm
        return m is not None and m.info.width > 0 and m.info.height > 0

    def lifecycle_state(self, node: str):
        """Lifecycle label of a Nav2 node ('active', 'inactive', ...), None when its get_state service does not answer."""
        cl = self._lc.get(node)
        if cl is None:
            cl = self._lc[node] = self.node.create_client(GetState, f"/{node}/get_state")
        if not cl.service_is_ready():
            return None
        res = self._wait(cl.call_async(GetState.Request()), 2.0)
        return res.current_state.label if res is not None else None

    def _pose_msg(self, x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = "map"
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        if yaw is None:
            p = self.pose()
            yaw = math.atan2(y - p[1], x - p[0]) if p else 0.0
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return m

    def nav_send(self, x, y, yaw):
        self._zero_until = 0.0
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_msg(x, y, yaw)
        self._h += 1
        h = self._h
        st = self._goals[h] = {"state": NAV_RUNNING, "dist": None, "recov": 0, "gh": None}

        def fb(m):
            st["dist"], st["recov"] = m.feedback.distance_remaining, m.feedback.number_of_recoveries

        def accepted(f):
            gh = f.result()
            if not gh.accepted:
                st["state"] = NAV_ABORTED
                return
            st["gh"] = gh
            gh.get_result_async().add_done_callback(lambda rf: st.__setitem__("state", _STATUS.get(rf.result().status, NAV_ABORTED)))

        self._nav.send_goal_async(goal, feedback_callback=fb).add_done_callback(accepted)
        return h

    def nav_state(self, h):
        g = self._goals[h]
        return {"state": g["state"], "distance_remaining": g["dist"], "recoveries": g["recov"]}

    def nav_cancel(self, h):
        g = self._goals[h]
        if g["gh"] is not None:
            g["gh"].cancel_goal_async()                  # also ends the BT's recovery behaviours (spin / backup / wait) of this goal
        if g["gh"] is None or g["state"] == NAV_RUNNING:
            g["state"] = NAV_CANCELED

    def compute_path(self, x, y, start=None):
        if not self._plan.server_is_ready():
            return None
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose_msg(x, y, None)
        goal.use_start = start is not None                 # default: from the robot's current pose
        if start is not None:
            goal.start = self._pose_msg(start[0], start[1], None)
        gh = self._wait(self._plan.send_goal_async(goal), 3.0)
        if gh is None or not gh.accepted:
            return None
        res = self._wait(gh.get_result_async(), 5.0)
        if res is None or res.status != 4 or not res.result.path.poses:
            return None
        pts = [(p.pose.position.x, p.pose.position.y) for p in res.result.path.poses]
        return {"length_m": sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])), "points": pts}

    # ---- exploration: the `explore` action of frontier_explorer_node (serve_action:=true)
    def explore_available(self) -> bool:
        return self._explore.server_is_ready()

    def explore_start(self, max_time_s, area_rect):
        goal = Explore.Goal()
        goal.max_time_s = float(max_time_s)
        goal.area_rect = [float(v) for v in (area_rect or [])]
        goal.return_to_start = False             # the agent decides what happens next (find / return_home)
        self._h += 1
        h = self._h
        st = self._explores[h] = {"state": NAV_RUNNING, "gh": None, "cancel": False}

        def accepted(f):
            gh = f.result()
            if not gh.accepted:
                st["state"] = NAV_ABORTED
                return
            st["gh"] = gh
            if st["cancel"]:                     # cancel was asked before the goal was accepted
                gh.cancel_goal_async()
            gh.get_result_async().add_done_callback(
                lambda rf: st.__setitem__("state", _STATUS.get(rf.result().status, NAV_ABORTED)))

        self._explore.send_goal_async(goal).add_done_callback(accepted)
        return h

    def explore_state(self, h):
        return {"state": self._explores[h]["state"]}

    def explore_cancel(self, h):
        st = self._explores[h]
        if st["state"] != NAV_RUNNING:
            return
        st["cancel"] = True
        if st["gh"] is not None:
            st["gh"].cancel_goal_async()

    # ---- actuation
    def hold_zero(self, seconds):
        """A stopped / failed mission must really stop: Nav2's smoother and recovery behaviours may still publish for a moment, so zero is
        published at 20 Hz for `seconds` (a new non-zero command or a new goal ends the hold)."""
        self._zero_until = time.monotonic() + float(seconds)
        self._pub_cmd.publish(Twist())

    def _zero_tick(self):
        if time.monotonic() < self._zero_until:
            self._pub_cmd.publish(Twist())

    def send_cmd(self, vx, vy, wz):
        if vx or vy or wz:
            self._zero_until = 0.0
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = float(vx), float(vy), float(wz)
        self._pub_cmd.publish(m)

    def set_estop(self, active):
        self._estop = bool(active)
        self._pub_estop.publish(Bool(data=self._estop))

    def estop_active(self) -> bool:
        return self._estop
