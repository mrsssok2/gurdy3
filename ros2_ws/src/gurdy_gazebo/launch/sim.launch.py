"""Spawn Gurdy in Gazebo Harmonic with ros2_control (effort PD at robot.yaml's pd_rate_hz).

    ros2 launch gurdy_gazebo sim.launch.py                    # GUI, robot standing on the ground
    ros2 launch gurdy_gazebo sim.launch.py gui:=false         # headless server (tests, less RAM)
    ros2 launch gurdy_gazebo sim.launch.py fixed_base:=true   # body bolted in the air (step tests)
    ros2 launch gurdy_gazebo sim.launch.py robot_yaml:=/path/variant.yaml   # modified robot

Everything numeric comes from robot.yaml via gurdy_core: controller gains/rate, initial stance,
spawn height. The controller parameter file is generated at launch time (not hand-written).
"""
import os
import tempfile

import xacro
import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from gurdy_core.kinematics import leg_ik, nominal_foot
from gurdy_core.robot import load_robot


def controllers_params(robot, js_rate: int) -> dict:
    """ros2_control parameters derived from robot.yaml (single source of truth).
    joint_state_broadcaster runs at js_rate (100 Hz default: 500 Hz /joint_states made
    robot_state_publisher burn ~27 % CPU); the PD controller runs at pd_rate_hz."""
    c = robot.control
    return {
        "controller_manager": {"ros__parameters": {
            "update_rate": int(c["pd_rate_hz"]),
            "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
            "joint_pd_controller": {"type": "gurdy_control/JointPDController"},
        }},
        "joint_state_broadcaster": {"ros__parameters": {"update_rate": int(js_rate)}},
        "joint_pd_controller": {"ros__parameters": {
            "joints": robot.joint_names,
            "kp": float(c["kp"]),
            "kd": float(c["kd"]),
            "effort_limit": float(robot.joint_common["effort"]),
        }},
    }


def arm_controllers(ctrl: dict) -> dict:
    """Phase 9 (arm:=true only): add the arm / gripper controllers of gurdy_arm_moveit/config/ros2_controllers.yaml to
    the controller-manager parameters (loaded lazily: without the arm nothing here runs)."""
    with open(os.path.join(get_package_share_directory("gurdy_arm_moveit"), "config", "ros2_controllers.yaml")) as f:
        extra = yaml.safe_load(f)
    out = dict(ctrl)
    out["controller_manager"] = {"ros__parameters": {**ctrl["controller_manager"]["ros__parameters"],
                                                     **extra["controller_manager"]["ros__parameters"]}}
    for k, v in extra.items():
        if k != "controller_manager":
            out[k] = v
    return out


WRIST_BRIDGE = ["/wrist_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/wrist_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/wrist_camera/trigger@std_msgs/msg/Bool]gz.msgs.Boolean"]


def _setup(context):
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    arm = LaunchConfiguration("arm").perform(context).lower() == "true"
    fixed = LaunchConfiguration("fixed_base").perform(context).lower() == "true"
    sensors = LaunchConfiguration("sensors").perform(context).lower() == "true"
    lidar = LaunchConfiguration("lidar").perform(context).lower() == "true"
    world = LaunchConfiguration("world").perform(context)

    desc_share = get_package_share_directory("gurdy_description")
    robot_yaml = LaunchConfiguration("robot_yaml").perform(context) or \
        os.path.join(desc_share, "config", "robot.yaml")
    robot = load_robot(robot_yaml)

    fd, ctrl_yaml = tempfile.mkstemp(prefix="gurdy_controllers_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        ctrl = controllers_params(robot, int(LaunchConfiguration("js_rate").perform(context)))
        yaml.safe_dump(arm_controllers(ctrl) if arm else ctrl, f)

    q0 = leg_ik(robot, nominal_foot(robot))
    base_z = 0.3 if fixed else robot.stance["body_height"] + 0.005
    urdf = xacro.process_file(
        os.path.join(desc_share, "urdf", "gurdy_with_arm.urdf.xacro" if arm else "gurdy.urdf.xacro"),
        mappings={"sim_gazebo": "true", "controllers_yaml": ctrl_yaml, "robot_yaml": robot_yaml,
                  **({"arm": "true", "arm_yaml": LaunchConfiguration("arm_yaml").perform(context)
                      or os.path.join(desc_share, "config", "arm.yaml")} if arm else {}),
                  "initial_q": " ".join(f"{v:.6f}" for v in q0),
                  "fixed_base": str(fixed).lower(), "base_z": str(base_z),
                  "sensors": str(sensors).lower(), "lidar": str(lidar).lower()},
    ).toxml()

    if not world.endswith(".sdf"):
        world = os.path.join(get_package_share_directory("gurdy_gazebo"), "worlds", world + ".sdf")
    # Headless: server only; cameras then need EGL headless rendering.
    gz_args = f"-r -v 2 {world}" + ("" if gui else " -s --headless-rendering")

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items())
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[{"robot_description": urdf, "use_sim_time": True}], output="screen")
    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", "gurdy",
                            "-z", "0.0" if fixed else str(base_z)])
    # bridge.yaml: clock + ground-truth odom always; sensor topics are lazy (cost nothing unless
    # subscribed, and absent in Gazebo when sensors:=false).
    bridge_cfg = os.path.join(get_package_share_directory("gurdy_gazebo"), "config", "bridge.yaml")
    clock = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                 parameters=[{"config_file": bridge_cfg, "use_sim_time": True}])
    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"])
    pd = Node(package="controller_manager", executable="spawner",
              arguments=["joint_pd_controller", "--controller-manager-timeout", "60"])
    actions = [
        gz, rsp, clock, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[pd])),
    ]
    if arm:      # Phase 9: arm + gripper controllers after the legs' PD, wrist camera topics + trigger
        arm_sp = Node(package="controller_manager", executable="spawner",
                      arguments=["arm_controller", "gripper_controller", "--controller-manager-timeout", "60"])
        actions += [RegisterEventHandler(OnProcessExit(target_action=pd, on_exit=[arm_sp])),
                    Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen", arguments=WRIST_BRIDGE,
                         parameters=[{"use_sim_time": True}])]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("fixed_base", default_value="false"),
        DeclareLaunchArgument("world", default_value="empty"),
        DeclareLaunchArgument("sensors", default_value="true", description="IMU, RGB-D camera, GT odom"),
        DeclareLaunchArgument("lidar", default_value="false", description="2D lidar on /scan (nav profile)"),
        DeclareLaunchArgument("js_rate", default_value="100", description="/joint_states rate (Hz)"),
        DeclareLaunchArgument("arm", default_value="false", description="Phase 9: spawn with the arm + gripper + wrist camera"),
        DeclareLaunchArgument("arm_yaml", default_value="", description="alternative arm.yaml (e.g. + payload mass)"),
        DeclareLaunchArgument("robot_yaml", default_value="",
                              description="alternative robot.yaml (experiments / fault injection)"),
        OpaqueFunction(function=_setup),
    ])
