# Copyright 2026 Tatsukiyano
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    pkg_bringup = get_package_share_directory("robot_bringup")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # --- 1. Load Global Config ---
    use_sim_time = LaunchConfiguration("gazebo").perform(context).lower() == 'true'
    actuator_type = LaunchConfiguration("actuator_type").perform(context) or "at"
    protocol = LaunchConfiguration("protocol").perform(context) or "at"

    paths = {
        "mux": os.path.join(pkg_bringup, "config", "twist_mux.yaml"),
        "teleop": os.path.join(pkg_bringup, "config", "teleop.yaml"),
        "ekf": os.path.join(pkg_bringup, "config", "ekf.yaml"),
        "controllers": os.path.join(pkg_bringup, "config", "controllers.yaml"),
        "communication": os.path.join(pkg_bringup, "config", "communication.yaml"),
        "xacro": os.path.join(pkg_bringup, "urdf", "robot.urdf.xacro"),
        "world": os.path.join(pkg_bringup, "world", "obstacles.sdf"),
        "bridge": os.path.join(pkg_bringup, "config", "gz_bridge.yaml"),
    }

    import xacro
    robot_description_xml = xacro.process_file(
        paths["xacro"], 
        mappings={"actuator_type": actuator_type, "is_gazebo": str(use_sim_time).lower(), "protocol": protocol}
    ).toxml()

    actions = []

    if use_sim_time:
        gz_args = f"-r -v 1 {paths['world']}"
        if LaunchConfiguration("headless").perform(context).lower() == 'true':
            gz_args = "-s " + gz_args

        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
            launch_arguments={'gz_args': gz_args}.items(),
        ))

        actions.append(Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'lazytatzv_robot', '-string', robot_description_xml],
            parameters=[{'use_sim_time': True}], output='screen'
        ))

        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='parameter_bridge',
            parameters=[{'config_file': paths["bridge"], 'use_sim_time': False}],
            output='screen'
        ))
    else:
        # Physical/Virtual Mode: Real ros2_control_node is required
        actions.append(Node(
            package="controller_manager", executable="ros2_control_node",
            parameters=[{'robot_description': robot_description_xml}, paths["controllers"]],
            output="screen",
        ))

        # --- Communication Bridge (Physical Mode only) ---
        if actuator_type == "at":
            if protocol == "at":
                # Generic Serial Bridge
                actions.append(Node(
                    package="serial_driver", executable="serial_driver_node",
                    name="serial_driver",
                    parameters=[paths["communication"]],
                    remappings=[("tx", "/communication/tx"), ("rx", "/communication/rx")],
                    output="screen",
                ))
            elif protocol == "can":
                # Seeed USB-CAN Analyzer Bridge
                actions.append(Node(
                    package="seeed_usb_can_analyzer_driver", executable="usb_can_analyzer_node",
                    name="usb_can_analyzer_node",
                    parameters=[paths["communication"]],
                    output="screen",
                ))

    # --- Controller Spawning ---
    # Gazebo mode: gz_ros2_control loads controllers automatically via <parameters> tag in URDF.
    # We only call spawner if NOT in Gazebo, or as a lightweight check.
    # To be safe and compatible with all modes, we use spawner but accept it might "fail" if already active.
    actions += [
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["mecanum_drive_controller"]),
    ]

    # --- Sensor Fusion ---
    actions.append(Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        parameters=[paths["ekf"], {'use_sim_time': use_sim_time}],
        remappings=[("odom0", "odom/wheels")], output='screen'
    ))

    # --- Teleop ---
    actions.append(ComposableNodeContainer(
        name="robot_core_container", namespace="", package="rclcpp_components", executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="base_teleop", plugin="base_teleop::BaseTeleopNode", name="teleop",
                parameters=[paths["teleop"], {"use_sim_time": use_sim_time}]
            )
        ], output="screen",
    ))
    # Note: Lifecycle manager is kept for teleop but autostarted directly
    actions.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_robot", 
                        parameters=[{"autostart": True, "node_names": ["teleop"], "bond_timeout": 4.0, "use_sim_time": use_sim_time}]))

    # --- Utilities ---
    actions += [
        Node(package="joy", executable="joy_node", name="joy_node", parameters=[paths["teleop"]]),
        Node(package="twist_mux", executable="twist_mux", name="twist_mux", 
             parameters=[paths["mux"], {"use_sim_time": use_sim_time}], 
             remappings=[("cmd_vel_out", "/mecanum_drive_controller/reference")]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher", 
             parameters=[{"robot_description": robot_description_xml, "publish_frequency": 20.0, "use_sim_time": use_sim_time}]),
    ]

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("actuator_type", default_value="at"),
        DeclareLaunchArgument("protocol", default_value="at"),
        DeclareLaunchArgument("gazebo", default_value="false"),
        DeclareLaunchArgument("headless", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
