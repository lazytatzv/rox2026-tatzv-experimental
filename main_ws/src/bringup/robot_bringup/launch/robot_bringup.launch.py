# Copyright 2026 Tatsukiyano
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    pkg_bringup = get_package_share_directory("robot_bringup")

    # Resolve configurations
    actuator_type = LaunchConfiguration("actuator_type").perform(context)
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_rviz = LaunchConfiguration("use_rviz")

    phys_config = os.path.join(pkg_bringup, "config", "physical.yaml")
    mux_config = os.path.join(pkg_bringup, "config", "twist_mux.yaml")
    teleop_config = os.path.join(pkg_bringup, "config", "teleop.yaml")
    urdf_path = os.path.join(pkg_bringup, "urdf", "robot.urdf")

    # --- 1. Dynamic Managed Nodes Logic ---
    # We must match the EXACT node names used in ComposableNode definitions
    managed_nodes = [
        "/hal/speed_dispatcher",
        "/mecanum_kinematics_node",
        "/motors/front_left",
        "/motors/front_right",
        "/motors/rear_left",
        "/motors/rear_right"
    ]
    if actuator_type != 'virtual':
        managed_nodes.append("/serial_driver")

    # --- 2. Setup Actuator Config ---
    if actuator_type == "at":
        m_pkg, m_plugin = "robstride_driver", "robstride_driver::RobstrideAtNode"
        act_yaml = os.path.join(pkg_bringup, "config", "actuators_robstride.yaml")
    elif actuator_type == "can":
        m_pkg, m_plugin = "robstride_driver", "robstride_driver::RobstrideCanNode"
        act_yaml = os.path.join(pkg_bringup, "config", "actuators_robstride.yaml")
    elif actuator_type == "ddsm":
        m_pkg, m_plugin = "ddsm115_ros2_driver", "ddsm115_ros2_driver::DDSM115DriverNode"
        act_yaml = os.path.join(pkg_bringup, "config", "actuators_ddsm.yaml")
    elif actuator_type == "virtual":
        m_pkg, m_plugin = "virtual_actuator", "virtual_actuator::VirtualActuatorNode"
        act_yaml = os.path.join(pkg_bringup, "config", "actuators_robstride.yaml")

    # --- 3. Composable Nodes ---
    control_nodes = [
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
            parameters=[act_yaml],
        ),
    ]

    for side in ["front_left", "front_right", "rear_left", "rear_right"]:
        control_nodes.append(
            ComposableNode(
                package=m_pkg,
                plugin=m_plugin,
                name=side,
                namespace="motors",
                parameters=[act_yaml, {"joint_name": f"{side}_wheel_joint"}],
            )
        )

    container = ComposableNodeContainer(
        name="actuator_control_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=control_nodes,
        output="screen",
    )

    # --- 4. Driver Nodes ---
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

    actions = [
        container,
        serial_driver,
        lifecycle_manager,
        Node(package="joy", executable="joy_node", name="joy_node", parameters=[teleop_config]),
        Node(
            package="base_teleop",
            executable="base_teleop_node",
            name="teleop",
            parameters=[teleop_config],
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            parameters=[mux_config],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": open(urdf_path).read()}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_aggregator",
            parameters=[{
                "source_list": [f"/motors/{s}/joint_states" for s in ["front_left", "front_right", "rear_left", "rear_right"]],
                "rate": 50,
            }],
        ),
    ]

    if actuator_type == 'at' or actuator_type == 'ddsm' or actuator_type == 'can':
         print(f"\n[MASTER LAUNCH] Mode: {actuator_type.upper()} (HARDWARE ACTIVE)")

    # --- 6. Optional Visualization ---
    try:
        foxglove_pkg = get_package_share_directory("foxglove_bridge")
        actions.append(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(foxglove_pkg, "launch", "foxglove_bridge_launch.xml")
            ),
            condition=IfCondition(use_foxglove)
        ))
    except Exception:
        pass

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("actuator_type", default_value="at"),
        DeclareLaunchArgument("use_foxglove", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
