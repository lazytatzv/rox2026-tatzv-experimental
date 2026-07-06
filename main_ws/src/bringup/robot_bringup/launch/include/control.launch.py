# Copyright 2026 Tatsukiyano
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import UnlessCondition


def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory("robot_bringup")
    tuning_config = os.path.join(pkg_robot_bringup, "config", "params", "tuning.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Auto-detect IMU
    from launch.substitutions import Command, PythonExpression
    check_imu_cmd = Command(['python3 ', os.path.join(pkg_robot_bringup, 'scripts', 'check_imu.py')])
    has_imu_arg = DeclareLaunchArgument('has_imu', default_value=check_imu_cmd)
    has_imu = LaunchConfiguration('has_imu')
    
    # If we do NOT have an IMU, fallback to Odom for Yaw
    use_odom_for_yaw = PythonExpression(["'true' if '", has_imu, "' == 'false' else 'false'"])

    # Container for all C++ components to achieve Zero-Copy & Low Latency
    container = ComposableNodeContainer(
        name="robot_control_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="imu_stabilizer",
                plugin="imu_stabilizer::HeadingStabilizerNode",
                name="heading_stabilizer",
                parameters=[
                    tuning_config, 
                    {
                        "use_sim_time": use_sim_time,
                        "use_odom_for_yaw": use_odom_for_yaw
                    }
                ],
                remappings=[
                    ("/cmd_vel_in", "/cmd_vel_teleop"),
                    ("/cmd_vel_out", "/mecanum_drive_controller/reference"),
                ],
            ),
            # 2. Mad Motor (Shooter) Command Node
            ComposableNode(
                package="mad_motor_driver",
                plugin="mad_motor_driver::MadMotorCommandNode",
                name="mad_motor_command",
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                    ("/shooter/cmd_muxed", "/shooter/cmd_muxed"),
                    ("/can_tx", "/can_tx"),
                ],
            ),
        ],
        output="screen",
    )

    # Bridge to physical CAN bus (Sender only, for Mad Motor)
    # Only launch on real hardware (when use_sim_time is false)
    socket_can_sender = Node(
        package="ros2_socketcan",
        executable="socket_can_sender_node",
        name="socket_can_sender",
        parameters=[{
            "interface": "can0",
            "use_sim_time": use_sim_time
        }],
        remappings=[
            ("to_can_bus", "/can_tx"),
        ],
        condition=UnlessCondition(use_sim_time)
    )

    return LaunchDescription([has_imu_arg, container, socket_can_sender])
