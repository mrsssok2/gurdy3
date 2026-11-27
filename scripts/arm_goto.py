#!/usr/bin/env python3
"""Move the arm to a named pose through arm_controller (FollowJointTrajectory), for watching it in the Gazebo GUI.
No MoveIt: the joint targets come from arm.yaml (folded, servo_view, ...) or, for pre_grasp, the analytic IK for a
small cube at (-0.27, 0) behind the robot (gurdy_core.arm.plan_grasp, the same target as the MoveIt check).

    python3 scripts/arm_goto.py folded|servo_view|servo_view_far|pre_grasp [seconds]     (inside the sim container)
    scripts/arm_pose.sh POSE                                                            (from the host)
"""
import sys
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gurdy_core"))
from gurdy_core import arm as A  # noqa: E402


def target(name):
    a = A.load_arm()
    if name == "pre_grasp":
        rb = A.load_robot()
        pg = A.plan_grasp(a, rb, A.PICKABLE["small_box"], (-0.27, 0.0), A.ground_z(rb))
        if not pg:
            sys.exit("no analytic pre-grasp for a cube at (-0.27, 0)")
        return pg[0]
    if name not in a.poses:
        sys.exit(f"unknown pose {name!r}; known: {sorted(a.poses)} + pre_grasp")
    return a.poses[name]


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "folded"
    secs = int(float(sys.argv[2])) if len(sys.argv) > 2 else 4
    q = [float(v) for v in target(name)]
    rclpy.init()
    n = rclpy.create_node("arm_goto")
    ac = ActionClient(n, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    if not ac.wait_for_server(timeout_sec=20.0):
        sys.exit("arm_controller action not available (is the sim running with arm:=true?)")
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = list(A.ARM_JOINTS)
    pt = JointTrajectoryPoint(positions=q)
    pt.time_from_start.sec = secs
    g.trajectory.points.append(pt)
    f = ac.send_goal_async(g)
    rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
    h = f.result()
    if h is None or not h.accepted:
        sys.exit("goal rejected")
    r = h.get_result_async()
    rclpy.spin_until_future_complete(n, r, timeout_sec=secs * 4 + 20)
    code = r.result().result.error_code if r.result() else None
    print(f"{name}: {' '.join(f'{v:+.3f}' for v in q)} rad -> {'done' if code == 0 else f'error code {code}'}")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
