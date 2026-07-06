# Copyright 2026 Tatsukiyano
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory("robot_bringup")
    default_config = os.path.join(pkg_robot_bringup, "config", "teleop_mux.yaml")
    analysis_config = os.path.join(pkg_robot_bringup, "config", "teleop_mux_analysis.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    analysis_mode = LaunchConfiguration("analysis_mode", default="false")
    config_file = PythonExpression(
        [
            "'",
            analysis_config,
            "' if '",
            analysis_mode,
            "' == 'true' else '",
            default_config,
            "'",
        ]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick_driver_node",
        parameters=[config_file, {"use_sim_time": use_sim_time}],
    )

    base_teleop_node = Node(
        package="base_teleop",
        executable="base_teleop_node",
        name="base_teleop",
        parameters=[config_file, {"use_sim_time": use_sim_time}],
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[config_file, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel_out", "/cmd_vel_teleop")],
    )

    foxglove_teleop_relay = Node(
        package="base_teleop",
        executable="twist_to_stamped",
        name="twist_to_stamped",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    shooter_teleop_node = Node(
        package="shooter_control",
        executable="shooter_teleop",
        name="shooter_teleop",
        parameters=[config_file, {"use_sim_time": use_sim_time}],
    )

    shooter_mux_node = Node(
        package="shooter_control",
        executable="shooter_mux",
        name="shooter_mux",
        parameters=[config_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("analysis_mode", default_value="false"),
            joy_node,
            base_teleop_node,
            twist_mux_node,
            foxglove_teleop_relay,
            shooter_teleop_node,
            shooter_mux_node,
        ]
    )
