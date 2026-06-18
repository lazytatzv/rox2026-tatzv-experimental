from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': 8765,
            'address': '0.0.0.0'
        }]
    )

    return LaunchDescription([
        foxglove_bridge
    ])
