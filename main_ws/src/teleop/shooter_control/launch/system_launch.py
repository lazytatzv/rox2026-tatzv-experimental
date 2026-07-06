import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('shooter_control'),
        'config',
        'system_params.yaml'
    )

    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy_node', output='screen',
             parameters=[{'autorepeat_rate': 20.0}]),
        Node(package='shooter_control', executable='shooter_teleop', name='shooter_teleop',
             output='screen', parameters=[config_file]),
        Node(package='shooter_control', executable='shooter_mux', name='shooter_mux',
             output='screen', parameters=[config_file]),
        Node(package='shooter_control', executable='auto_shooter', name='auto_shooter',
             output='screen', parameters=[config_file]),
        Node(package='shooter_control', executable='spring_controller', name='spring_controller',
             output='screen', parameters=[config_file]),
        Node(package='mad_motor_driver', executable='mad_motor_driver_node', name='mad_motor_driver_node',
             output='screen', parameters=[config_file])
    ])
