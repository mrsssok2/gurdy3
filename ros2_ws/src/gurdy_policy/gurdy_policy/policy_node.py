"""RL policy node (step 2.9).

    /cmd_vel (Twist)  ┐
    /imu (Imu)        ├─> gurdy_core.policy_runtime.PolicyController (same obs as training, ONNX, EMA)
    /joint_states     ┘      at 50 Hz  ->  /joint_pd_controller/commands (18 joint targets)

Same interface as gait_node (run ONE of them): everything above speaks only /cmd_vel.
Timing is driven by /joint_states stamps (sim time, 100 Hz), one policy tick per 20 ms of sim time
(catch-up like gait_node), so it needs no use_sim_time.

Parameters
  mode          "policy" (default) | "tripod": switch at runtime (`ros2 param set /policy_node mode tripod`);
                the tripod mode is the baseline gait inside the same node (A/B on one launch).
  model         path to the .onnx (a .json with the same stem holds the metadata);
                default $GURDY_REPO|~/gurdy2 / models / terrain_dr2_15M.onnx
  cmd_timeout   s of sim time without /cmd_vel   -> policy mode: ramp to the stand pose (tripod: zero command)
  sensor_timeout  s without a fresh /imu (sim time) or /joint_states (wall time) -> stand pose
  stand_speed   rad/s joint speed limit while moving to the stand pose
  /estop (std_msgs/Bool, latched): true -> go to the stand pose (joint speed limited, like a timeout), ignore /cmd_vel and hold it
                until /estop is false again. Works in both modes (policy and tripod). The stand pose is the safe pose: all
                legs under the body, no stride, no free-falling limbs.
After a stand period the controller is reset (zero history/previous action, clock phase 0) = the
rest start the policy was trained from.

Topics out: /joint_pd_controller/commands, /policy/cmd (Twist: command after deadband/clip, as fed to
the policy), /policy/status (String: policy | stand:<reason> | tripod).
"""
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64MultiArray, String

from gurdy_core.gait import TripodGait
from gurdy_core.policy_runtime import OnnxPolicy, PolicyController, load_meta
from gurdy_core.robot import load_robot

DEFAULT_MODEL = "terrain_dr2_15M.onnx"


def _stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class PolicyNode(Node):
    def __init__(self):
        super().__init__("policy_node")
        self.declare_parameter("robot_yaml", "")
        self.declare_parameter("model", "")
        self.declare_parameter("mode", "policy")
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("sensor_timeout", 0.5)
        self.declare_parameter("stand_speed", 1.0)
        self.declare_parameter("zero_cmd_stand_s", 0.1)   # an exactly-zero /cmd_vel for this long (sim s) = "stop": ramp to the stand pose at once (0 = off)
        self.robot = load_robot(self.get_parameter("robot_yaml").value or None)
        model = self.get_parameter("model").value or os.path.join(
            os.environ.get("GURDY_REPO", os.path.expanduser("~/gurdy2")), "models", DEFAULT_MODEL)
        self.meta = load_meta(model)
        self.policy = OnnxPolicy(model)
        self.ctl = PolicyController(self.robot, self.policy, self.meta)
        self.gait = TripodGait(self.robot)
        self.dt = self.ctl.dt
        self.q_stance = self.ctl.q_stance
        self.index = {n: i for i, n in enumerate(self.robot.joint_names)}
        self.mode = self._check_mode(self.get_parameter("mode").value)
        self.add_on_set_parameters_callback(self._on_params)

        self.cmd = np.zeros(3)
        self.last_cmd_time = None           # sim time of the last /cmd_vel
        self.zero_since = None              # sim time since which every /cmd_vel was exactly zero (the agent's stop sends zeros)
        self.q = self.qd = None
        self.imu = None                     # (quat_wxyz, gyro)
        self.imu_time = None                # sim stamp of the last /imu
        self.sim_now = None
        self.last_js_wall = None
        self.last_tick = None
        self.q_out = self.q_stance.copy()   # last published target (stand ramp starts here)
        self.in_stand = True
        self.estop = False
        self.status = ""
        self.t_infer = []

        self.pub = self.create_publisher(Float64MultiArray, "/joint_pd_controller/commands", 10)
        self.pub_cmd = self.create_publisher(Twist, "/policy/cmd", 10)
        self.pub_status = self.create_publisher(String, "/policy/status", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, "/estop", self.on_estop, latched)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Imu, "/imu", self.on_imu, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.create_timer(0.25, self.watchdog)      # wall-clock: /joint_states stopped -> stand pose
        self.get_logger().info(
            f"policy node: mode={self.mode} model={os.path.basename(model)} "
            f"(checkpoint {self.meta['checkpoint']}) {1 / self.dt:.0f} Hz")

    # ------------------------------------------------------------------ params / inputs
    def _check_mode(self, mode):
        if mode not in ("policy", "tripod"):
            raise ValueError(f"mode must be 'policy' or 'tripod', got {mode!r}")
        return mode

    def _on_params(self, params):
        for p in params:
            if p.name == "mode":
                if p.value not in ("policy", "tripod"):
                    return SetParametersResult(successful=False, reason="mode: policy | tripod")
                self.mode = p.value
                self.ctl.reset()
                self.gait = TripodGait(self.robot)
                self.last_cmd_time = None      # stand until a fresh /cmd_vel; switch while stationary
                self.get_logger().info(f"mode -> {self.mode}")
        return SetParametersResult(successful=True)

    def on_estop(self, msg: Bool):
        if bool(msg.data) != self.estop:
            self.get_logger().warn("E-STOP engaged: standing" if msg.data else "e-stop released")
            self.estop = bool(msg.data)
            self.cmd = np.zeros(3)
            self.last_cmd_time = None          # after a release, stand until a fresh /cmd_vel arrives
            self.ctl.reset()
            self.gait = TripodGait(self.robot)

    def on_cmd(self, msg: Twist):
        if self.estop:
            return
        self.cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        self.last_cmd_time = self.sim_now
        if not self.cmd.any():
            self.zero_since = self.sim_now if self.zero_since is None else self.zero_since
        else:
            self.zero_since = None

    def on_imu(self, msg: Imu):
        o, w = msg.orientation, msg.angular_velocity
        self.imu = (np.array([o.w, o.x, o.y, o.z]), np.array([w.x, w.y, w.z]))
        self.imu_time = _stamp(msg)

    def on_joint_states(self, msg: JointState):
        t = _stamp(msg)
        self.last_js_wall = time.monotonic()
        if len(msg.velocity) == len(msg.name):
            q, qd = np.zeros(18), np.zeros(18)
            for n, p, v in zip(msg.name, msg.position, msg.velocity):
                i = self.index.get(n)
                if i is not None:
                    q[i], qd[i] = p, v
            self.q, self.qd = q, qd
        if self.last_tick is not None and t < self.last_tick:      # sim reset: start over
            self.last_tick = None
            self.ctl.reset()
        self.sim_now = t
        if self.last_tick is None:
            self.last_tick = t
            self.tick()
            return
        n = int((t - self.last_tick + 1e-6) // self.dt)             # catch up like gait_node
        if n > 0:
            self.last_tick += n * self.dt
            if n > 1:
                self.get_logger().info("late /joint_states: caught up", throttle_duration_sec=10.0)
            self.tick()

    # ------------------------------------------------------------------ control
    def stand_reason(self):
        """None if the policy may run, else why we must stand."""
        if self.estop:
            return "estop"
        if self.q is None:
            return "no joint_states"
        if self.imu is None or self.sim_now - self.imu_time > float(self.get_parameter("sensor_timeout").value):
            return "imu timeout"
        if self.last_cmd_time is None or self.sim_now - self.last_cmd_time > float(self.get_parameter("cmd_timeout").value):
            return "cmd_vel timeout"
        z = float(self.get_parameter("zero_cmd_stand_s").value)
        if z > 0 and self.zero_since is not None and self.sim_now - self.zero_since >= z:
            return "stop command"                     # the policy would still finish its stride; standing stops the body sooner
        return None

    def publish_targets(self, q):
        self.q_out = q
        self.pub.publish(Float64MultiArray(data=q.tolist()))

    def set_status(self, s):
        if s != self.status:
            self.status = s
            self.pub_status.publish(String(data=s))
            self.get_logger().info(f"status: {s}")

    def go_stand(self, reason):
        """Move to the stand pose with a joint speed limit; arms a controller reset."""
        step = float(self.get_parameter("stand_speed").value) * self.dt
        self.publish_targets(self.q_out + np.clip(self.q_stance - self.q_out, -step, step))
        if not self.in_stand:
            self.in_stand = True
        self.ctl.reset()
        self.set_status(f"stand:{reason}")

    def tick(self):
        if self.estop:                                  # both modes: safe stand pose, no stride
            self.go_stand("estop")
            return
        if self.mode == "tripod":                       # baseline gait: zero command on timeout, as gait_node
            fresh = self.last_cmd_time is not None and self.sim_now - self.last_cmd_time <= float(
                self.get_parameter("cmd_timeout").value)
            self.publish_targets(self.gait.step(self.cmd if fresh else np.zeros(3)))
            self.set_status("tripod")
            return
        reason = self.stand_reason()
        if reason:
            self.go_stand(reason)
            return
        t0 = time.perf_counter()
        q = self.ctl.step(self.imu[0], self.imu[1], self.q, self.qd, self.cmd)
        self.t_infer.append(time.perf_counter() - t0)
        self.in_stand = False
        self.publish_targets(q)
        c = self.ctl.command
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = float(c[0]), float(c[1]), float(c[2])
        self.pub_cmd.publish(m)
        self.set_status("policy")
        if len(self.t_infer) >= 500:
            a = np.array(self.t_infer) * 1e3
            self.get_logger().info(f"policy tick (obs+ONNX+mapping): mean {a.mean():.2f} ms, max {a.max():.2f} ms "
                                   f"over {len(a)} ticks")
            self.t_infer.clear()

    def watchdog(self):
        """/joint_states silent for sensor_timeout wall seconds (sim paused or crashed): stand pose."""
        if self.last_js_wall is None or (self.mode != "policy" and not self.estop):
            return
        if time.monotonic() - self.last_js_wall > float(self.get_parameter("sensor_timeout").value):
            self.go_stand("joint_states timeout")


def main():
    rclpy.init()
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
