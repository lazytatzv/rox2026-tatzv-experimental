# Copyright 2026 Tatsukiyano
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_bringup = get_package_share_directory("robot_bringup")

    # --- 1. Arguments ---
    actuator_type_arg = DeclareLaunchArgument("actuator_type", default_value="at")
    use_foxglove_arg = DeclareLaunchArgument("use_foxglove", default_value="true")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="false")

    actuator_type = LaunchConfiguration("actuator_type")
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_rviz = LaunchConfiguration("use_rviz")

    # Paths
    phys_config = os.path.join(pkg_bringup, "config", "physical.yaml")
    mux_config = os.path.join(pkg_bringup, "config", "twist_mux.yaml")
    teleop_config = os.path.join(pkg_bringup, "config", "teleop.yaml")
    urdf_path = os.path.join(pkg_bringup, "urdf", "robot.urdf")

    # --- 2. Dynamic Managed Nodes Logic ---
    managed_nodes = [
        "/hal/speed_dispatcher",
        "/mecanum_kinematics_node",
        "/motors/front_left",
        "/motors/front_right",
        "/motors/rear_left",
        "/motors/rear_right"
    ]

    # --- 3. Composable Nodes (HAL & Logic) ---
    container = ComposableNodeContainer(
        name="actuator_control_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="mecanum_kinematics",
                plugin="mecanum_kinematics::MecanumKinematicsNode",
                name="mecanum_kinematics_node",
                parameters=[phys_config],
            ),
            ComposableNode(
                package="mecanum_kinematics",
                plugin="mecanum_kinematics::WheelSpeedsDispatcher",
                name="speed_dispatcher",
                namespace="hal",
                parameters=[os.path.join(pkg_bringup, "config", "actuators_robstride.yaml")],
            ),
        ],
        output="screen",
    )

    # --- 4. Driver Nodes (Conditional) ---
    serial_driver = Node(
        package="serial_driver",
        executable="serial_bridge",
        name="serial_driver",
        parameters=[{
            "device_name": "/dev/ttyUSB1",
            "baud_rate": 921600,
            "flow_control": "none",
            "parity": "none",
            "stop_bits": "1"
        }],
        remappings=[("write", "/serial_write"), ("read", "/serial_read")],
        condition=UnlessCondition(PythonExpression(["'", actuator_type, "' == 'virtual'"])),
        output="screen"
    )

    # --- 5. Base System Nodes ---
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_robot",
        parameters=[{"autostart": True, "node_names": managed_nodes, "bond_timeout": 0.0}],
    )

    joy_node = Node(package="joy", executable="joy_node", name="joy_node", parameters=[teleop_config])
    
    # We no longer use 'remappings' here. Instead, we ensure teleop.yaml defines the correct topic names.
    teleop_node = Node(
        package="base_teleop",
        executable="base_teleop_node",
        name="teleop",
        parameters=[teleop_config],
    )

    # Ensure twist_mux.yaml defines the final output topic as 'cmd_vel'
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[mux_config],
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": open(urdf_path).read()}],
    )

    joint_aggregator = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_aggregator",
        parameters=[{
            "source_list": [f"/motors/{s}/joint_states" for s in ["front_left", "front_right", "rear_left", "rear_right"]],
            "rate": 50,
        }],
    )

    # --- 6. Optional Visualization ---
    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml")
        ),
        condition=IfCondition(use_foxglove)
    )

    rviz2 = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        actuator_type_arg,
        use_foxglove_arg,
        use_rviz_arg,
        container,
        serial_driver,
        lifecycle_manager,
        joy_node,
        teleop_node,
        twist_mux,
        robot_state_pub,
        joint_aggregator,
        foxglove_bridge,
        rviz2
    ])
