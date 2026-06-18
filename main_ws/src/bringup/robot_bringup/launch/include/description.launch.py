# Copyright 2026 Tatsukiyano
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    urdf_path = os.path.join(pkg_robot_bringup, 'urdf', 'robot.urdf.xacro')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo = LaunchConfiguration('gazebo')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware', default='false')

    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path, ' gazebo:=', gazebo, ' use_mock_hardware:=', use_mock_hardware]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher
    ])
