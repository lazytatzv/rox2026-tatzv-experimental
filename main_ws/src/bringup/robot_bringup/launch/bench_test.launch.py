# Copyright 2026 Tatsukiyano
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_bringup')

    # Arguments
    motor_id = LaunchConfiguration('motor_id', default='1')
    usb_path = LaunchConfiguration('usb_path', default='/dev/ttyUSB0')

    urdf_path = os.path.join(pkg_robot_bringup, 'urdf', 'robot.urdf.xacro')
    controllers_config = os.path.join(pkg_robot_bringup, 'config', 'bench_controllers.yaml')

    # Robot State Publisher (Single Joint Mode)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ', urdf_path,
                ' bench_mode:=true',
                ' bench_motor_id:=', motor_id,
                ' usb_path:=', usb_path
            ]),
            'use_sim_time': False
        }]
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': Command([
                'xacro ', urdf_path,
                ' bench_mode:=true',
                ' bench_motor_id:=', motor_id,
                ' usb_path:=', usb_path
            ])},
            controllers_config
        ],
        output='screen'
    )

    # Spawner
    spawn_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['bench_joint_controller', '--controller-manager-timeout', '30']
    )

    return LaunchDescription([
        DeclareLaunchArgument('motor_id', default_value='1'),
        DeclareLaunchArgument('usb_path', default_value='/dev/ttyUSB0'),
        robot_state_publisher,
        controller_manager,
        spawn_controller
    ])
