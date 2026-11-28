"""Gurdy in Gazebo, walking: simulator + controllers + tripod gait node (+ optional RViz).

    ros2 launch gurdy_bringup sim.launch.py                      # Gazebo GUI, sensors on
    ros2 launch gurdy_bringup sim.launch.py gui:=false sensors:=false rviz:=true
    ros2 launch gurdy_bringup sim.launch.py controller:=policy      # RL policy (ONNX) instead of the tripod gait
    ros2 launch gurdy_bringup sim.launch.py world:=semantic_lab perception:=true target_frame:=odom   # Phase 4 detector
    ros2 launch gurdy_bringup sim.launch.py world:=semantic_lab perception:=true target_frame:=odom object3d:=true   # Phase 8c 3D positions
Mapping / navigation (Phase 3; adds the lidar, flat_lab world, odom TF, slam_toolbox or AMCL, Nav2):
    ros2 launch gurdy_bringup sim.launch.py nav:=slam                       # map while driving (teleop or Nav2 goals)
    ros2 launch gurdy_bringup sim.launch.py nav:=localization map:=flat_lab # saved map (maps/flat_lab.yaml) + AMCL
    (nav:=slam|localization|none, scan_source:=lidar|depth, odom_source:=ground_truth|ekf, navigation:=false for SLAM only, controller:=tripod|policy)
Drive it (second terminal: docker compose exec sim bash):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.3
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _nav_setup(context):
    """nav:=slam|localization: choose world/lidar defaults, add the navigation launch."""
    nav = context.launch_configurations["nav"]
    if nav == "none":
        return []
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("gurdy_navigation"), "launch", "nav.launch.py")),
        launch_arguments={"mode": nav, "navigation": LaunchConfiguration("navigation"),
                          "scan_source": LaunchConfiguration("scan_source"), "map": LaunchConfiguration("map"),
                          "robot_yaml": LaunchConfiguration("robot_yaml"), "rviz": LaunchConfiguration("rviz"),
                          "odom_source": LaunchConfiguration("odom_source")}.items())]


def _gazebo_setup(context):
    """The nav profile switches on the lidar (scan_source:=lidar) and, unless given, the flat_lab world."""
    nav = context.launch_configurations["nav"] != "none"
    lc = context.launch_configurations
    lidar = "true" if (nav and lc["scan_source"] == "lidar") else lc["lidar"]
    world = lc["world"] or ("flat_lab" if nav else "empty")
    sensors = "true" if (nav or lc["perception"] == "true") else lc["sensors"]   # depth source, IMU, camera need sensors
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("gurdy_gazebo"), "launch", "sim.launch.py")),
        launch_arguments={"gui": lc["gui"], "sensors": sensors, "lidar": lidar, "world": world,
                          "robot_yaml": lc["robot_yaml"], "js_rate": lc["js_rate"], "arm": lc.get("arm", "false"),
                          "arm_yaml": lc.get("arm_yaml", "")}.items())]


def generate_launch_description():
    args = ["gui", "sensors", "world", "robot_yaml", "js_rate"]
    defaults = {"gui": "true", "sensors": "true", "world": "", "robot_yaml": "", "js_rate": "100"}   # world "" = auto
    sim = OpaqueFunction(function=_gazebo_setup)
    controller = LaunchConfiguration("controller")     # tripod (gait_node) | policy (policy_node, needs sensors:=true)
    gate = LaunchConfiguration("estop_gate")           # true: /cmd_vel -> estop_gate_node -> /cmd_vel_safe -> controller
    shield = LaunchConfiguration("shield")             # true: /cmd_vel -> safety_shield_node -> /cmd_vel_shielded -> (gate)
    shielded = PythonExpression(["'/cmd_vel_shielded' if '", shield, "' == 'true' else '/cmd_vel'"])
    # E-STOP gate stays LAST (closest to the controller): /cmd_vel [-> shield] [-> gate] -> controller
    cmd_in = PythonExpression(["'/cmd_vel_safe' if '", gate, "' == 'true' else ('/cmd_vel_shielded' if '", shield,
                               "' == 'true' else '/cmd_vel')"])
    estop_gate = Node(package="gurdy_safety", executable="estop_gate_node", output="screen", remappings=[("/cmd_vel", shielded)],
                      condition=IfCondition(PythonExpression(["'", gate, "' == 'true'"])))
    safety_shield = Node(package="gurdy_safety", executable="safety_shield_node", output="screen",
                         condition=IfCondition(PythonExpression(["'", shield, "' == 'true'"])),
                         parameters=[{"robot_yaml": LaunchConfiguration("robot_yaml")}])
    gait = Node(package="gurdy_gait", executable="gait_node", output="screen", remappings=[("/cmd_vel", cmd_in)],
                condition=IfCondition(PythonExpression(["'", controller, "' == 'tripod'"])),
                parameters=[{"robot_yaml": LaunchConfiguration("robot_yaml")}])   # timed by /joint_states stamps
    policy = Node(package="gurdy_policy", executable="policy_node", output="screen", remappings=[("/cmd_vel", cmd_in)],
                  condition=IfCondition(PythonExpression(["'", controller, "' == 'policy'"])),
                  parameters=[{"robot_yaml": LaunchConfiguration("robot_yaml"), "model": LaunchConfiguration("model")}])
    perception = Node(package="gurdy_perception", executable="detector_node", output="screen",
                      condition=IfCondition(LaunchConfiguration("perception")),
                      parameters=[{"model": LaunchConfiguration("detector_model"), "use_sim_time": False,
                                   "target_frame": LaunchConfiguration("target_frame"),
                                   "log_path": LaunchConfiguration("detector_log"),
                                   "ttl": PythonExpression(["float(", LaunchConfiguration("detector_ttl"), ")"]),
                                   "position_method": PythonExpression(["'object3d' if '", LaunchConfiguration("object3d"),
                                                                        "' == 'true' else 'pixel'"])}])
    rec_dir = LaunchConfiguration("costmap_record_dir")     # Phase 8a cause 2: costmap + pose at each planner failure
    costmap_rec = Node(package="gurdy_navigation", executable="costmap_recorder_node", output="screen",
                       condition=IfCondition(PythonExpression(["'", rec_dir, "' != ''"])),
                       parameters=[{"out_dir": rec_dir, "use_sim_time": True}])
    rviz = Node(package="rviz2", executable="rviz2", condition=IfCondition(PythonExpression(
                    ["'", LaunchConfiguration("rviz"), "' == 'true' and '", LaunchConfiguration("nav"), "' == 'none'"])),
                arguments=["-d", os.path.join(get_package_share_directory("gurdy_description"), "rviz", "display.rviz")],
                parameters=[{"use_sim_time": True}])
    return LaunchDescription(
        [DeclareLaunchArgument(a, default_value=defaults[a]) for a in args]
        + [DeclareLaunchArgument("rviz", default_value="false"),
           DeclareLaunchArgument("controller", default_value="tripod"),
           DeclareLaunchArgument("nav", default_value="none", description="none | slam | localization"),
           DeclareLaunchArgument("navigation", default_value="true", description="start Nav2 (with nav:=slam|localization)"),
           DeclareLaunchArgument("odom_source", default_value="ground_truth", description="ground_truth | ekf (leg odometry + IMU)"),
           DeclareLaunchArgument("scan_source", default_value="lidar", description="lidar | depth"),
           DeclareLaunchArgument("lidar", default_value="false", description="force the lidar on without nav"),
           DeclareLaunchArgument("map", default_value="", description="localization: map yaml or name in maps/"),
           OpaqueFunction(function=_nav_setup),
           DeclareLaunchArgument("perception", default_value="false", description="Phase 4: detector + semantic map (camera on)"),
           DeclareLaunchArgument("detector_model", default_value="", description="ONNX in models/detector/ (default det_v31_s.onnx; rollback det_v3_s.onnx / det_v2_s.onnx)"),
           DeclareLaunchArgument("detector_log", default_value="", description="JSON-lines log of every detector sighting"),
           DeclareLaunchArgument("detector_ttl", default_value="30.0",
                                 description="s an unconfirmed detector track is kept after its last sighting (was 10)"),
           DeclareLaunchArgument("object3d", default_value="false",
                                 description="Phase 8c: detector positions from all box depth pixels + floor fit + shape fit (default: Phase 4 pixel method)"),
           DeclareLaunchArgument("target_frame", default_value="map", description="frame of the semantic map (odom without SLAM)"),
           DeclareLaunchArgument("estop_gate", default_value="false", description="cmd_vel e-stop gate (gurdy_safety) in front of the controller"),
           DeclareLaunchArgument("arm", default_value="false", description="Phase 9: arm + gripper + wrist camera (gurdy_with_arm)"),
           DeclareLaunchArgument("arm_yaml", default_value="", description="Phase 9: alternative arm.yaml"),
           DeclareLaunchArgument("shield", default_value="false",
                                 description="Phase 8b LiDAR safety shield (CBF) on /cmd_vel; needs a /scan (nav:=slam|localization)"),
           DeclareLaunchArgument("costmap_record_dir", default_value="",
                                 description="Phase 8a: save the global costmap + pose at every planner failure into this dir (empty = off)"),
           DeclareLaunchArgument("model", default_value=""), sim, gait, policy, estop_gate, safety_shield, perception, costmap_rec, rviz])
