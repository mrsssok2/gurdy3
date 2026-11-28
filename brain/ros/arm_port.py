"""ArmPort: what the ROS fetch robot (brain/ros/fetch_robot.py) needs from the arm side, as plain Python types.

Two implementations: `rclpy_arm_port.RclpyArmPort` (real topics / actions; NOT run yet) and `fake_arm_port.FakeArmPort`
(tests: kinematic arm, synthetic wrist depth, contact-like grasp rules, over brain.ros.fake_port.FakePort). Joint vectors
are the 5 arm joints of gurdy_core.arm.ARM_JOINTS (rad). The body (Nav2, /cmd_vel, pose, e-stop) stays in RosPort.
"""
from __future__ import annotations

from typing import Protocol


class ArmPort(Protocol):
    def arm_ready(self) -> bool: ...                                  # arm_controller (+ gripper_controller) answering
    def moveit_available(self) -> bool: ...                           # /move_action answering
    def arm_q(self) -> list[float] | None: ...                        # current arm joints (/joint_states)
    def move_arm(self, qs: list, seconds: float, use_moveit: bool = False) -> bool: ...   # one goal (qs = [q]) or waypoints; blocks
    def gripper(self, effort: float) -> None: ...                     # effort per finger, N (+ = open, - = close/squeeze)
    def fingers(self) -> tuple[float, float] | None: ...              # finger joint positions (m, opening per finger)
    def efforts(self) -> dict | None: ...                               # arm joint efforts (N m), averaged ~0.5 s; None = unknown
    def wrist_view(self) -> tuple | None: ...                         # trigger + fresh frame: (depth (H, W) float32, K, T_base_opt) or None
    def set_speed_limit(self, mps: float | None) -> None: ...         # Nav2 controller speed limit (None = no limit)
    def wait(self, seconds: float) -> None: ...                       # let time pass (wall sleep on the real robot)
    def settle(self, q: list, tol: float = 0.003, timeout: float = 3.0) -> float: ...   # wait for the arm at q; -> max joint error (rad)
    def add_floor(self, margin: float = 0.005) -> dict: ...           # fitted floor plane -> MoveIt scene ({'added': bool, ...})
    def target_in_scene(self, cls: str, xy_base, on: bool = True) -> bool: ...   # grasp target as a MoveIt obstacle (big moves only)
