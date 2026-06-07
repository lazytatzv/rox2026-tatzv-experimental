from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "el05_usb_can_driver"

    config_file = LaunchConfiguration("config_file")
    usb_can_config_file = LaunchConfiguration("usb_can_config_file")
    start_usb_can = LaunchConfiguration("start_usb_can")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare(package_name), "config", "el05_usb_can.yaml"]
                ),
                description="Path to the EL05 driver parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "usb_can_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare(package_name),
                        "config",
                        "usb_can_analyzer_el05.yaml",
                    ]
                ),
                description="Path to the Seeed USB-CAN analyzer parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "start_usb_can",
                default_value="true",
                description="Start the Seeed USB-CAN analyzer node from this launch file.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("seeed_usb_can_analyzer_driver"),
                            "launch",
                            "usb_can_analyzer.launch.py",
                        ]
                    )
                ),
                launch_arguments={"config_file": usb_can_config_file}.items(),
                condition=IfCondition(start_usb_can),
            ),
            Node(
                package=package_name,
                executable="el05_motor_node",
                name="el05_motor_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
