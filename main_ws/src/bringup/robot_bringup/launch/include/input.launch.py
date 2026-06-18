import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    config_file = os.path.join(pkg_robot_bringup, 'config', 'teleop_mux.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick_driver_node',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )

    base_teleop_node = Node(
        package='base_teleop',
        executable='base_teleop_node',
        name='base_teleop',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/cmd_vel_teleop')]
    )

    return LaunchDescription([
        joy_node,
        base_teleop_node,
        twist_mux_node
    ])
