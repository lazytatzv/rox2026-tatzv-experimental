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

    # Resolve arguments
    is_gazebo = LaunchConfiguration("gazebo").perform(context).lower() == 'true'
    is_headless = LaunchConfiguration("headless").perform(context).lower() == 'true'
    actuator_type = LaunchConfiguration("actuator_type").perform(context)
    use_foxglove = LaunchConfiguration("use_foxglove")

    # Configuration paths
    paths = {
        "phys": os.path.join(pkg_bringup, "config", "physical.yaml"),
        "mux": os.path.join(pkg_bringup, "config", "twist_mux.yaml"),
        "teleop": os.path.join(pkg_bringup, "config", "teleop.yaml"),
        "urdf": os.path.join(pkg_bringup, "urdf", "robot.urdf"),
    }

    if is_gazebo:
        print(f"\n[MASTER LAUNCH] Mode: GAZEBO (High-Fidelity Physics)")
    else:
        print(f"\n[MASTER LAUNCH] Mode: {actuator_type.upper()}")

    actions = []

    # --- 1. Gazebo Simulation Mode ---
    if is_gazebo:
        actions.append(SetEnvironmentVariable('GZ_IP', '127.0.0.1'))
        
        world_path = os.path.join(pkg_bringup, "world", "obstacles.sdf")
        print(f"[SIM] Loading Obstacle World: {world_path}")
        gz_args = f"-r -v 1 {world_path}"
        if is_headless:
            gz_args = "-s " + gz_args

        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
            launch_arguments={'gz_args': gz_args}.items(),
        ))

        actions.append(Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'lazytatzv_robot', '-file', paths["urdf"]],
            output='screen'
        ))

        # Strict Unidirectional Bridges to prevent multiple channel conflicts
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',    # ROS -> GZ
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',     # GZ -> ROS
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',        # GZ -> ROS
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'      # GZ -> ROS
            ],
            output='screen'
        ))

        # Vision: Broadcast Gazebo obstacles as ROS Markers
        actions.append(Node(
            package='mecanum_kinematics', executable='obstacle_visualizer_node',
            name='obstacle_visualizer', output='screen'
        ))

    # --- 2. ROS 2 Actuator/Logic Mode (Non-Gazebo) ---
    else:
        # Build managed nodes list
        managed_nodes = [
            "/hal/speed_dispatcher",
            "/mecanum_kinematics_node",
            "/motors/front_left",
            "/motors/front_right",
            "/motors/rear_left",
            "/motors/rear_right"
        ]
        
        if actuator_type == "at" or actuator_type == "virtual":
            m_pkg, m_plugin = "robstride_driver", "robstride_driver::RobstrideAtNode"
            if actuator_type == "virtual":
                m_pkg, m_plugin = "virtual_actuator", "virtual_actuator::VirtualActuatorNode"
            act_yaml = os.path.join(pkg_bringup, "config", "actuators_robstride.yaml")
        elif actuator_type == "ddsm":
            m_pkg, m_plugin = "ddsm115_ros2_driver", "ddsm115_ros2_driver::DDSM115DriverNode"
            act_yaml = os.path.join(pkg_bringup, "config", "actuators_ddsm.yaml")

        control_nodes = [
            ComposableNode(
                package="mecanum_kinematics",
                plugin="mecanum_kinematics::MecanumKinematicsNode",
                name="mecanum_kinematics_node",
                parameters=[paths["phys"], {"topic_cmd_vel": "/cmd_vel", "topic_wheel_speeds": "/hal/wheel_speeds"}],
            ),
            ComposableNode(
                package="mecanum_kinematics",
                plugin="mecanum_kinematics::WheelSpeedsDispatcher",
                name="speed_dispatcher",
                namespace="hal",
                parameters=[act_yaml, {"subscription_topic": "/hal/wheel_speeds"}],
            ),
        ]

        for side in ["front_left", "front_right", "rear_left", "rear_right"]:
            control_nodes.append(
                ComposableNode(
                    package=m_pkg, plugin=m_plugin, name=side, namespace="motors",
                    parameters=[act_yaml, {"joint_name": f"{side}_wheel_joint", "topic_tx_queue": "/serial_write", "topic_rx_queue": "/serial_read"}],
                )
            )

        actions.append(ComposableNodeContainer(
            name="actuator_control_container", namespace="", package="rclcpp_components",
            executable="component_container", composable_node_descriptions=control_nodes,
            output="screen",
        ))

        actions.append(Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_robot",
            parameters=[{"autostart": True, "node_names": managed_nodes, "bond_timeout": 0.0}],
        ))

        if actuator_type != 'virtual':
            actions.append(Node(
                package="serial_driver", executable="serial_bridge", name="serial_driver",
                parameters=[{
                    "device_name": "/dev/ttyUSB1", "baud_rate": 921600,
                    "flow_control": "none", "parity": "none", "stop_bits": "1"
                }],
                remappings=[("write", "/serial_write"), ("read", "/serial_read")],
                output="screen"
            ))

    # --- 3. Base UI & System (Always Active) ---
    actions += [
        Node(package="joy", executable="joy_node", name="joy_node", parameters=[paths["teleop"]]),
        Node(package="mecanum_kinematics", executable="zero_twist_node", name="zero_twist_node"),
        Node(package="base_teleop", executable="base_teleop_node", name="teleop", parameters=[paths["teleop"]]),
        Node(package="twist_mux", executable="twist_mux", name="twist_mux", 
             parameters=[paths["mux"]], remappings=[("cmd_vel_out", "/cmd_vel")]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
            parameters=[{
                "robot_description": open(paths["urdf"]).read(),
                "publish_frequency": 20.0  # Constant re-broadcast
            }]),
    ]

    # Optional Visualization
    if LaunchConfiguration("use_foxglove").perform(context).lower() == 'true':
        try:
            foxglove_pkg = get_package_share_directory("foxglove_bridge")
            actions.append(IncludeLaunchDescription(
                AnyLaunchDescriptionSource(os.path.join(foxglove_pkg, "launch", "foxglove_bridge_launch.xml"))
            ))
        except Exception:
            pass

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("actuator_type", default_value="at"),
        DeclareLaunchArgument("gazebo", default_value="false"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("use_foxglove", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
