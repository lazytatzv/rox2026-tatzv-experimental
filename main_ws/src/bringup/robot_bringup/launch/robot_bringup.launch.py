import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    
    # 1. Config Paths
    tuning_config = os.path.join(pkg_robot_bringup, 'config', 'params', 'tuning.yaml')

    # 2. Declare Arguments (Defaults to Real Hardware)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    gazebo = LaunchConfiguration('gazebo', default='false')
    headless = LaunchConfiguration('headless', default='false')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware', default='false')

    # 3. Modular Launch Includes
    include_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_robot_bringup, 'launch', 'include', 'description.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time, 'gazebo': gazebo, 'use_mock_hardware': use_mock_hardware}.items()
    )

    include_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_robot_bringup, 'launch', 'include', 'localization.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time, 'gazebo': gazebo}.items()
    )

    include_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_robot_bringup, 'launch', 'include', 'sim.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time, 'gazebo': gazebo, 'headless': headless}.items()
    )

    include_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_robot_bringup, 'launch', 'include', 'control.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    include_input = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_robot_bringup, 'launch', 'include', 'input.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. Spawners & High-level Logic
    spawn_broadcaster = Node(
        package='controller_manager', executable='spawner', 
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120']
    )
    spawn_controller = Node(
        package='controller_manager', executable='spawner', 
        arguments=['mecanum_drive_controller', '--controller-manager-timeout', '120'],
        remappings=[('/mecanum_drive_controller/cmd_vel', '/cmd_vel_stabilized')]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gazebo', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        include_description,
        include_localization,
        include_sim,
        include_control,
        include_input,
        spawn_broadcaster,
        spawn_controller
    ])
