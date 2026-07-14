# Copyright 2026 Tatsukiyano
#
# Minimal teleop-only launch for manual mecanum drive control.
# Launches only what is needed: URDF, ros2_control, joystick, and teleop.
# Sensors, localization, navigation, and IMU stabilizer are NOT started.
#
# Usage:
#   ros2 launch robot_bringup teleop_only.launch.py
#   (or via Justfile: just teleop)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory("robot_bringup")
    teleop_config = os.path.join(pkg_robot_bringup, "config", "teleop_mux.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    gazebo = LaunchConfiguration("gazebo", default="false")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware", default="false")

    # --- 1. Robot Description (URDF / TF) ---
    urdf_path = os.path.join(pkg_robot_bringup, "urdf", "robot.urdf.xacro")
    controllers_config = os.path.join(pkg_robot_bringup, "config", "controllers.yaml")

    robot_description_content = ParameterValue(
        Command(
            ["xacro ", urdf_path, " gazebo:=", gazebo, " use_mock_hardware:=", use_mock_hardware]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": use_sim_time}],
    )

    # --- 2. ros2_control (Robstride AT serial driver) ---
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content, "use_sim_time": use_sim_time},
            controllers_config,
        ],
        output="screen",
    )

    # --- 3. Controller Spawners ---
    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "120"],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "--controller-manager-timeout", "120"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # --- 4. Joystick Input ---
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick_driver_node",
        parameters=[teleop_config, {"use_sim_time": use_sim_time}],
    )

    # --- 5. Base Teleop (Joy -> TwistStamped) ---
    base_teleop_node = Node(
        package="base_teleop",
        executable="base_teleop_node",
        name="base_teleop",
        parameters=[teleop_config, {"use_sim_time": use_sim_time}],
    )

    # --- 6. Twist Mux ---
    #     Output remapped directly to mecanum_drive_controller/reference.
    #     twist_mux (use_stamped: true) publishes TwistStamped, which
    #     mecanum_drive_controller (use_stamped_vel: true) expects on ~/reference.
    #     No heading stabilizer or relay needed.
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[teleop_config, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel_out", "/mecanum_drive_controller/reference")],
    )

    # --- 7. Foxglove Bridge (for web monitoring) ---
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{"use_sim_time": use_sim_time, "port": 8765, "address": "0.0.0.0"}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("gazebo", default_value="false"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            robot_state_publisher,
            ros2_control_node,
            spawn_broadcaster,
            spawn_controller,
            joy_node,
            base_teleop_node,
            twist_mux_node,
            foxglove_bridge,
        ]
    )
