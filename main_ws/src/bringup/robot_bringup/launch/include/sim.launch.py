import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo = LaunchConfiguration('gazebo')

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r ' + os.path.join(pkg_robot_bringup, 'world', 'rox2026_field.sdf')}.items(),
        condition=IfCondition(gazebo)
    )

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'rox2026', '-topic', 'robot_description'],
        condition=IfCondition(gazebo)
    )

    return LaunchDescription([
        gazebo_sim,
        spawn_robot
    ])
