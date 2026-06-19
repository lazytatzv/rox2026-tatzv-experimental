# Copyright 2026 Tatsukiyano
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
    EmitEvent,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_control_analysis = get_package_share_directory("control_analysis")
    default_config = os.path.join(pkg_control_analysis, "config", "analysis_settings.yaml")

    mode = LaunchConfiguration("mode")
    bag_name = LaunchConfiguration("bag_name")
    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_str = context.perform_substitution(use_sim_time)
    is_sim = use_sim_time_str.lower() in ["true", "1", "yes"]

    # Conditions for selecting which node to run
    # Mode "auto" triggers auto_analyzer. Any other mode triggers signal_injector.
    condition_auto = IfCondition(PythonExpression(["'", mode, "' == 'auto'"]))
    condition_injector = UnlessCondition(PythonExpression(["'", mode, "' == 'auto'"]))

    # 2. Automated Bag Recording
    # Records control input, wheel state, localization output, and ground truth
    cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        bag_name,
        "/cmd_vel_ext",
        "/cmd_vel_teleop",
        "/mecanum_drive_controller/reference",
        "/odometry/filtered",
        "/odom/ground_truth",
        "/joint_states",
    ]
    if is_sim:
        cmd.append("--use-sim-time")

    bag_record = ExecuteProcess(
        cmd=cmd,
        output="screen",
    )

    # 3. Nodes Configuration
    # Auto Analyzer Node (runs automated chirp + step sequence)
    auto_analyzer_node = Node(
        package="control_analysis",
        executable="auto_analyzer",
        name="auto_analyzer",
        parameters=[config_file, {"use_sim_time": use_sim_time}],
        output="screen",
        condition=condition_auto,
    )

    # Signal Injector Node (runs step, sine, or chirp based on config)
    signal_injector_node = Node(
        package="control_analysis",
        executable="signal_injector",
        name="signal_injector",
        parameters=[config_file, {"mode": mode, "use_sim_time": use_sim_time}],
        output="screen",
        condition=condition_injector,
    )

    # 4. Event Handlers for Clean Termination
    # When the test node exits, we shut down the entire launch description,
    # which automatically stops the bag recording cleanly with SIGINT.
    shutdown_on_injector_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=signal_injector_node,
            on_exit=[
                LogInfo(
                    msg="[Control Analysis] Signal injector finished. Terminating bag recording..."
                ),
                EmitEvent(event=Shutdown(reason="Signal injector test complete")),
            ],
        ),
        condition=condition_injector,
    )

    shutdown_on_analyzer_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=auto_analyzer_node,
            on_exit=[
                LogInfo(
                    msg="[Control Analysis] Auto analyzer finished. Terminating bag recording..."
                ),
                EmitEvent(event=Shutdown(reason="Auto analyzer test complete")),
            ],
        ),
        condition=condition_auto,
    )

    return [
        bag_record,
        auto_analyzer_node,
        signal_injector_node,
        shutdown_on_injector_exit,
        shutdown_on_analyzer_exit,
    ]


def generate_launch_description():
    pkg_control_analysis = get_package_share_directory("control_analysis")
    default_config = os.path.join(pkg_control_analysis, "config", "analysis_settings.yaml")

    # 1. Declare Launch Arguments
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="step",
        description='Control test mode: "step", "sine", "chirp", or "auto"',
    )
    bag_name_arg = DeclareLaunchArgument(
        "bag_name",
        default_value="control_analysis_bag",
        description="Output directory/name of the recorded rosbag",
    )
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to the YAML configuration file for node parameters",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    return LaunchDescription(
        [
            mode_arg,
            bag_name_arg,
            config_file_arg,
            use_sim_time_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
