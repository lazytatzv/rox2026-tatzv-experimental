import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # パラメータファイルのパスを取得
    config_file = os.path.join(
        get_package_share_directory('shooter_control'),
        'config',
        'spring_params.yaml'
    )

    return LaunchDescription([
        # Joyノードの起動
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'autorepeat_rate': 20.0}] # 必要に応じてレート調整
        ),
        
        # 最強のばね発射コントローラノードの起動
        Node(
            package='shooter_control',
            executable='spring_controller',
            name='spring_controller',
            output='screen',
            parameters=[config_file]
        )
    ])
