# Copyright 2026 Tatsukiyano
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Pro Tip: Look for config in bringup package first
    pkg_bringup = get_package_share_directory("robot_bringup")
    config = os.path.join(pkg_bringup, "config", "sensors.yaml")

    return LaunchDescription(
        [
            Node(
                package="bno055_driver",
                executable="bno055_node",
                name="bno055_node",
                parameters=[config],
                output="screen",
            )
        ]
    )
