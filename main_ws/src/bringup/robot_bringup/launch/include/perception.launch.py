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

    pc_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        remappings=[
            ("cloud_in", "/camera/depth/points"),
            ("scan", "/scan"),
        ],
        parameters=[{
            "target_frame": "camera_link",
            "transform_tolerance": 0.01,
            "min_height": 0.0,
            "max_height": 1.0,
            "angle_min": -1.309,  # -75 degrees
            "angle_max": 1.309,   # 75 degrees
            "angle_increment": 0.0087, # 0.5 degrees
            "scan_time": 0.03333,
            "range_min": 0.1,
            "range_max": 10.0,
            "use_inf": True,
            "inf_epsilon": 1.0
        }]
    )

    tag_localization = Node(
        package="vision_localization",
        executable="tag_localizer",
        name="tag_localization_node",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            apriltag_node,
            pc_to_laserscan,
            tag_localization,
        ]
    )
