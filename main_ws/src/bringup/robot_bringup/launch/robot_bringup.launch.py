# Copyright 2026 Tatsukiyano
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    pkg_bringup = get_package_share_directory("robot_bringup")

    # 1. Resolve arguments at runtime
    actuator_type = LaunchConfiguration("actuator_type").perform(context)
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_rviz = LaunchConfiguration("use_rviz")

    # Configuration paths
    paths = {
        "phys": os.path.join(pkg_bringup, "config", "physical.yaml"),
        "mux": os.path.join(pkg_bringup, "config", "twist_mux.yaml"),
        "teleop": os.path.join(pkg_bringup, "config", "teleop.yaml"),
        "urdf": os.path.join(pkg_bringup, "urdf", "robot.urdf"),
    }

    # 2. Dynamic Managed Nodes Logic
    # We match the absolute node names for lifecycle management
    managed_nodes = [
        "/hal/speed_dispatcher",
        "/mecanum_kinematics_node",
        "/motors/front_left",
        "/motors/front_right",
        "/motors/rear_left",
        "/motors/rear_right"
    ]
    # Standard serial driver doesn't support lifecycle, so we don't manage it

    print(f"\n[MASTER LAUNCH] Mode: {actuator_type.upper()}")

    # 3. Setup Actuator Specifics
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
    else:
        raise RuntimeError(f"Unknown actuator_type: {actuator_type}")

    # 4. Composable Container (HAL & Kinematics)
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
                # Hardcode absolute topic for reliability
                parameters=[paths["phys"], {"topic_cmd_vel": "/cmd_vel", "topic_wheel_speeds": "/hal/wheel_speeds"}],
            ),
            ComposableNode(
                package="mecanum_kinematics",
                plugin="mecanum_kinematics::WheelSpeedsDispatcher",
                name="speed_dispatcher",
                namespace="hal",
                parameters=[act_yaml, {"subscription_topic": "/hal/wheel_speeds"}],
            ),
        ],
        output="screen",
    )

    # Individual motor components
    for side in ["front_left", "front_right", "rear_left", "rear_right"]:
        container.composable_node_descriptions.append(
            ComposableNode(
                package=m_pkg,
                plugin=m_plugin,
                name=side,
                namespace="motors",
                parameters=[act_yaml, {"joint_name": f"{side}_wheel_joint", "topic_tx_queue": "/serial_write", "topic_rx_queue": "/serial_read"}],
            )
        )

    # 5. Core System Nodes
    actions = [
        container,
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_robot",
            parameters=[{"autostart": True, "node_names": managed_nodes, "bond_timeout": 0.0}],
        ),
        Node(package="joy", executable="joy_node", name="joy_node", parameters=[paths["teleop"]]),
        Node(
            package="base_teleop",
            executable="base_teleop_node",
            name="teleop",
            parameters=[paths["teleop"]],
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            parameters=[paths["mux"]],
            remappings=[("cmd_vel_out", "/cmd_vel")],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": open(paths["urdf"]).read()}],
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

    # Standard Serial Driver (Optional based on type)
    if actuator_type != 'virtual':
        actions.append(Node(
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
            output="screen"
        ))

    # Optional Visualization
    if use_foxglove.perform(context).lower() == 'true':
        try:
            foxglove_pkg = get_package_share_directory("foxglove_bridge")
            actions.append(IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(foxglove_pkg, "launch", "foxglove_bridge_launch.xml")
                )
            ))
        except Exception:
            pass

    if use_rviz.perform(context).lower() == 'true':
        actions.append(Node(package="rviz2", executable="rviz2", name="rviz2"))

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("actuator_type", default_value="at"),
        DeclareLaunchArgument("use_foxglove", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
