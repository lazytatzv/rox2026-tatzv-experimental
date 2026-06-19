import launch
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # 使用するインターフェース（実機で使うときは 'can0' に書き換えてください）
    can_interface = 'vcan0'

    # 1. LifecycleNode としてノードを定義
    receiver_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socket_can_receiver',
        namespace='',
        parameters=[{'interface': can_interface, 'enable_can_fd': False}],
        output='screen'
    )

    sender_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_sender_node_exe',
        name='socket_can_sender',
        namespace='',
        parameters=[{
            'interface': can_interface, 
            'enable_can_fd': False, 
            'timeout_sec': 0.01
        }],
        output='screen'
    )

    # 2. 起動直後に「Configure（準備）」イベントを発火させる設定
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

    # 3. Configureが完了して「Inactive（待機）」状態になったのを検知したら、
    # 自動的に「Activate（稼働）」イベントを発火させる設定
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

    # 以上の設定をすべてLaunchの実行リストに登録
    return LaunchDescription([
        receiver_node,
        sender_node,
        configure_receiver,
        configure_sender,
        activate_receiver,
        activate_sender,
    ])
