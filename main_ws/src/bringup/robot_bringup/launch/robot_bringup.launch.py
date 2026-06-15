import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    
    # 1. Declare Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gazebo = LaunchConfiguration('gazebo', default='true')

    # 2. Modular Launch Includes
    include_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_robot_bringup, 'launch', 'include', 'description.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    include_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_robot_bringup, 'launch', 'include', 'localization.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time, 'gazebo': gazebo}.items()
    )

    include_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_robot_bringup, 'launch', 'include', 'sim.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time, 'gazebo': gazebo}.items()
    )

    # 3. Dedicated Nodes (Control & Spawners)
    heading_stabilizer = Node(
        package='imu_stabilizer',
        executable='stabilizer_node',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel_teleop'),
            ('/cmd_vel_out', '/cmd_vel_stabilized')
        ]
    )

    spawn_broadcaster = Node(
        package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']
    )
    spawn_controller = Node(
        package='controller_manager', executable='spawner', 
        arguments=['mecanum_drive_controller'],
        remappings=[('/mecanum_drive_controller/cmd_vel', '/cmd_vel_stabilized')]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gazebo', default_value='true'),
        include_description,
        include_localization,
        include_sim,
        heading_stabilizer,
        spawn_broadcaster,
        spawn_controller
    ])
