import os
from glob import glob

from setuptools import setup

package_name = "gurdy_safety"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="S. Mahadev",
    description="Simulated battery, leg health and alert monitor for the Gurdy agent",
    license="MIT",
    entry_points={"console_scripts": [
        "battery_sim_node = gurdy_safety.battery_sim_node:main",
        "leg_health_node = gurdy_safety.leg_health_node:main",
        "alert_monitor_node = gurdy_safety.alert_monitor_node:main",
        "estop_gate_node = gurdy_safety.estop_gate_node:main",
        "sim_clock_node = gurdy_safety.sim_clock_node:main",
        "safety_shield_node = gurdy_safety.safety_shield_node:main"]},
)
