import os
from glob import glob

from setuptools import setup

package_name = "gurdy_navigation"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="S. Mahadev",
    description="Mapping (slam_toolbox) and navigation (Nav2) for Gurdy",
    license="MIT",
    entry_points={"console_scripts": [
        "odom_tf_node = gurdy_navigation.odom_tf_node:main",
        "leg_odometry_node = gurdy_navigation.leg_odometry_node:main",
        "cmd_vel_min_speed_node = gurdy_navigation.cmd_vel_min_speed_node:main",
        "scan_self_filter_node = gurdy_navigation.scan_self_filter_node:main",
        "frontier_explorer_node = gurdy_navigation.frontier_explorer_node:main",
        "low_obstacle_node = gurdy_navigation.low_obstacle_node:main",
        "costmap_recorder_node = gurdy_navigation.costmap_recorder_node:main"]},
)
