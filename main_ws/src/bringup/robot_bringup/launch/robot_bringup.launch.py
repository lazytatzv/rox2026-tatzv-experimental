import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Paths
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_bno055 = get_package_share_directory('bno055_driver')
    pkg_stabilizer = get_package_share_directory('imu_stabilizer')
    
    urdf_path = os.path.join(pkg_robot_bringup, 'urdf', 'robot.urdf.xacro')
    controllers_config = os.path.join(pkg_robot_bringup, 'config', 'controllers.yaml')
    ekf_config = os.path.join(pkg_robot_bringup, 'config', 'ekf.yaml')
    sensors_config = os.path.join(pkg_robot_bringup, 'config', 'sensors.yaml')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gazebo = LaunchConfiguration('gazebo', default='true')

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': use_sim_time
        }]
    )

    # 2. IMU Driver (Real only)
    imu_driver = Node(
        package='bno055_driver',
        executable='bno055_node',
        condition=UnlessCondition(gazebo),
        parameters=[sensors_config, {'use_sim_time': use_sim_time}]
    )

    # 3. EKF (Fuses Odometry + IMU + AprilTag)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odometry/filtered')]
    )

    # 4. Heading Stabilizer (The "Balancing" layer)
    heading_stabilizer = Node(
        package='imu_stabilizer',
        executable='stabilizer_node',
        name='imu_stabilizer',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel_teleop'),      # From Joystick
            ('/cmd_vel_out', '/cmd_vel_stabilized') # To Controller
        ]
    )

    # 5. Controllers (Gazebo or Real)
    # [Logic for Gazebo vs Physical hardware manager]
    # For now, focus on the wiring for simulation
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}, controllers_config],
        condition=UnlessCondition(gazebo)
    )

    spawn_broadcaster = Node(
        package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']
    )
    spawn_controller = Node(
        package='controller_manager', executable='spawner', 
        arguments=['mecanum_drive_controller', '--param-file', controllers_config],
        remappings=[('/mecanum_drive_controller/cmd_vel', '/cmd_vel_stabilized')]
    )

    # Gazebo Sim
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
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gazebo', default_value='true'),
        robot_state_publisher,
        imu_driver,
        ekf_node,
        heading_stabilizer,
        gazebo_sim,
        spawn_robot,
        spawn_broadcaster,
        spawn_controller
    ])
