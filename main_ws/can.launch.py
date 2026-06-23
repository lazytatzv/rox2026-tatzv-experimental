import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # Declare Launch Arguments
    declare_can_driver = DeclareLaunchArgument(
        'can_driver',
        default_value='socketcan',
        description='CAN driver to use: "socketcan" or "usb_can_analyzer"'
    )
    
    declare_can_interface = DeclareLaunchArgument(
        'interface',
        default_value='vcan0',
        description='SocketCAN interface name (only used for socketcan)'
    )

    declare_usb_path = DeclareLaunchArgument(
        'usb_path',
        default_value='/dev/ttyUSB1',
        description='USB serial path (only used for usb_can_analyzer)'
    )

    declare_serial_baud = DeclareLaunchArgument(
        'serial_baud',
        default_value='2000000',
        description='Serial baud rate (only used for usb_can_analyzer)'
    )

    declare_bitrate = DeclareLaunchArgument(
        'bitrate',
        default_value='500000',
        description='CAN bus bitrate'
    )

    can_driver = LaunchConfiguration('can_driver')
    can_interface = LaunchConfiguration('interface')
    usb_path = LaunchConfiguration('usb_path')
    serial_baud = LaunchConfiguration('serial_baud')
    bitrate = LaunchConfiguration('bitrate')

    # Resolve configuration file path
    try:
        pkg_robot_bringup = get_package_share_directory('robot_bringup')
        hardware_nodes_config = os.path.join(pkg_robot_bringup, 'config', 'hardware_nodes.yaml')
    except Exception:
        hardware_nodes_config = os.path.join(
            os.path.dirname(__file__),
            'src', 'bringup', 'robot_bringup', 'config', 'hardware_nodes.yaml'
        )

    # --- [ 1. SocketCAN Configurations ] ---
    receiver_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socket_can_receiver',
        namespace='',
        parameters=[
            hardware_nodes_config,
            {'interface': can_interface, 'enable_can_fd': False}
        ],
        output='screen'
    )

    sender_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_sender_node_exe',
        name='socket_can_sender',
        namespace='',
        parameters=[
            hardware_nodes_config,
            {
                'interface': can_interface, 
                'enable_can_fd': False, 
                'timeout_sec': 0.01
            }
        ],
        output='screen'
    )

    configure_receiver = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(receiver_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )
    
    configure_sender = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(sender_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_receiver = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=receiver_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(receiver_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    activate_sender = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=sender_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(sender_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    socketcan_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", can_driver, "' == 'socketcan'"])),
        actions=[
            receiver_node,
            sender_node,
            configure_receiver,
            configure_sender,
            activate_receiver,
            activate_sender,
        ]
    )

    # --- [ 2. USB-CAN Analyzer Configurations ] ---
    usb_can_node = LifecycleNode(
        package='seeed_usb_can_analyzer_driver',
        executable='usb_can_analyzer_node',
        name='usb_can_analyzer_node',
        namespace='',
        parameters=[
            hardware_nodes_config,
            {
                'usb_path': usb_path,
                'serial_baud': serial_baud,
                'bitrate': bitrate,
                'can_rx_topic': '/from_can_bus',
                'can_tx_topic': '/to_can_bus',
            }
        ],
        output='screen'
    )

    configure_usb_can = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(usb_can_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_usb_can = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=usb_can_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(usb_can_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    usb_can_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", can_driver, "' == 'usb_can_analyzer'"])),
        actions=[
            usb_can_node,
            configure_usb_can,
            activate_usb_can,
        ]
    )

    return LaunchDescription([
        declare_can_driver,
        declare_can_interface,
        declare_usb_path,
        declare_serial_baud,
        declare_bitrate,
        socketcan_group,
        usb_can_group,
    ])
