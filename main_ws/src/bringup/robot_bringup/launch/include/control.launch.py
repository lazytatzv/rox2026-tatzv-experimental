# Copyright 2026 Tatsukiyano
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory("robot_bringup")
    tuning_config = os.path.join(pkg_robot_bringup, "config", "params", "tuning.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Container for all C++ components to achieve Zero-Copy & Low Latency
    container = ComposableNodeContainer(
        name="robot_control_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # 1. Heading Stabilizer Component
            ComposableNode(
                package="imu_stabilizer",
                plugin="imu_stabilizer::HeadingStabilizerNode",
                name="heading_stabilizer",
                parameters=[tuning_config, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("/cmd_vel_in", "/cmd_vel_teleop"),
                    ("/cmd_vel_out", "/mecanum_drive_controller/reference"),
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([container])
