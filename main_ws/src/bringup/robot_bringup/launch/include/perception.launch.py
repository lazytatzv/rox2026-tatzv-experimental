import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_node",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"family": "16h5"},
            {"size": 0.15},
        ],
        remappings=[
            ("image_rect", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            apriltag_node,
        ]
    )
