"""Detector + semantic map (needs the camera: sim with sensors:=true, and TF to the target frame).

    ros2 launch gurdy_perception perception.launch.py model:=det_v3_s.onnx      (default; rollback: model:=det_v2_s.onnx)
    ros2 launch gurdy_bringup sim.launch.py nav:=slam world:=semantic_lab perception:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="", description="ONNX path or name in models/detector/"),
        DeclareLaunchArgument("target_frame", default_value="map"),
        DeclareLaunchArgument("providers", default_value="auto", description="auto | CPUExecutionProvider | CUDAExecutionProvider"),
        DeclareLaunchArgument("save_path", default_value="", description="write the semantic map JSON here on shutdown"),
        Node(package="gurdy_perception", executable="detector_node", output="screen",
             parameters=[{"model": LaunchConfiguration("model"), "target_frame": LaunchConfiguration("target_frame"),
                          "providers": LaunchConfiguration("providers"), "save_path": LaunchConfiguration("save_path")}]),
    ])
