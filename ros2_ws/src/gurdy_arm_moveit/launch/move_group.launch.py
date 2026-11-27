"""Phase 9e: MoveIt 2 move_group for the Gurdy arm. NOT run yet (needs the ROS image with MoveIt 2 + pick_ik).

    ros2 launch gurdy_arm_moveit move_group.launch.py arm:=true [use_sim_time:=true] [rviz:=true]
rviz:=true also opens RViz2 (config/moveit.rviz: robot, TF, /scan, /map, semantic markers, wrist image, MotionPlanning panel for
the `arm` group) with the same MoveIt parameters as move_group (needs the image gurdy3/ros:jazzy-moveit).
With arm:=false (default) nothing is started. MoveIt gets its OWN robot_description: gurdy_with_arm.urdf.xacro arm:=true
moveit_collision:=true (leg collision proxies so the planner sees the legs); Gazebo / robot_state_publisher keep the
description without proxies (same joints, so /joint_states fits both). The floor is added to the planning scene at run
time from the fitted floor plane (gurdy_core.low_obstacles.fit_floor; measured at map z = -0.11 m), not from a fixed number.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _yaml(pkg, rel):
    with open(os.path.join(get_package_share_directory(pkg), rel)) as f:
        return yaml.safe_load(f)


def _setup(context):
    if LaunchConfiguration("arm").perform(context) != "true":
        return [LogInfo(msg="gurdy_arm_moveit: arm:=false -> move_group not started")]
    import xacro
    desc = get_package_share_directory("gurdy_description")
    urdf = xacro.process_file(os.path.join(desc, "urdf", "gurdy_with_arm.urdf.xacro"), mappings={
        "arm": "true", "moveit_collision": "true",
        "robot_yaml": os.path.join(desc, "config", "robot.yaml"), "arm_yaml": os.path.join(desc, "config", "arm.yaml")}).toxml()
    share = get_package_share_directory("gurdy_arm_moveit")
    with open(os.path.join(share, "config", "gurdy.srdf")) as f:
        srdf = f.read()
    ompl = _yaml("gurdy_arm_moveit", "config/ompl_planning.yaml")
    params = {
        "robot_description": urdf,
        "robot_description_semantic": srdf,
        "robot_description_kinematics": _yaml("gurdy_arm_moveit", "config/kinematics.yaml"),
        "robot_description_planning": _yaml("gurdy_arm_moveit", "config/joint_limits.yaml"),
        "planning_pipelines": {"pipeline_names": ["ompl"]},
        "ompl": ompl,
        **_yaml("gurdy_arm_moveit", "config/moveit_controllers.yaml"),
        "publish_planning_scene": True, "publish_geometry_updates": True, "publish_state_updates": True,
        "publish_transforms_updates": True, "monitor_dynamics": False,
        "use_sim_time": LaunchConfiguration("use_sim_time").perform(context) == "true",
    }
    nodes = [Node(package="moveit_ros_move_group", executable="move_group", output="screen", parameters=[params])]
    if LaunchConfiguration("rviz").perform(context) == "true":
        rv = {k: params[k] for k in ("robot_description", "robot_description_semantic", "robot_description_kinematics",
                                     "robot_description_planning", "use_sim_time")}
        nodes.append(Node(package="rviz2", executable="rviz2", output="screen", parameters=[rv],
                          arguments=["-d", os.path.join(share, "config", "moveit.rviz")]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm", default_value="false", description="Phase 9e: start move_group for the arm"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false", description="also open RViz2 with the MotionPlanning panel"),
        OpaqueFunction(function=_setup),
    ])
