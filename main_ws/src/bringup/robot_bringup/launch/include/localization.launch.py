import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    ekf_config = os.path.join(pkg_robot_bringup, 'config', 'ekf.yaml')
    sensors_config = os.path.join(pkg_robot_bringup, 'config', 'sensors.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo = LaunchConfiguration('gazebo')

    imu_driver = Node(
        package='bno055_driver',
        executable='bno055_node',
        condition=UnlessCondition(gazebo),
        parameters=[sensors_config, {'use_sim_time': use_sim_time}]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odometry/filtered')]
    )

    return LaunchDescription([
        imu_driver,
        ekf_node
    ])
