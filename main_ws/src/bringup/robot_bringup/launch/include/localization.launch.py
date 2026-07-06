# Copyright 2026 Tatsukiyano
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition


def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory("robot_bringup")
    ekf_config = os.path.join(pkg_robot_bringup, "config", "ekf.yaml")
    hardware_nodes_config = os.path.join(pkg_robot_bringup, "config", "hardware_nodes.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gazebo = LaunchConfiguration("gazebo")

    # Auto-detect IMU
    check_imu_cmd = Command(['python3 ', os.path.join(pkg_robot_bringup, 'scripts', 'check_imu.py')])
    has_imu_arg = DeclareLaunchArgument('has_imu', default_value=check_imu_cmd)
    has_imu = LaunchConfiguration('has_imu')

    # Condition: NOT gazebo AND has_imu is true
    from launch.conditions import IfCondition
    from launch.substitutions import PythonExpression
    
    launch_imu_cond = IfCondition(PythonExpression(["'", has_imu, "' == 'true' and '", gazebo, "' == 'false'"]))

    imu_driver = Node(
        package="bno055_driver",
        executable="bno055_node",
        condition=launch_imu_cond,
        parameters=[hardware_nodes_config, {"use_sim_time": use_sim_time}],
    )

    local_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="local_ekf_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}],
        remappings=[("/odometry/filtered", "/odometry/local")],
    )

    global_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="global_ekf_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}],
        remappings=[("/odometry/filtered", "/odometry/filtered")],
    )

    return LaunchDescription([has_imu_arg, imu_driver, local_ekf_node, global_ekf_node])
