# Copyright 2026 Tatsukiyano
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # 1つ目のカメラ (BisonCam)
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="camera1",  # ノード名をシンプルに
                parameters=[
                    {
                        "video_device": "/dev/video0",
                    }
                ],
                # 🟢 トピックが衝突しないように、名前の頭に /camera1 を強制付与する
                remappings=[
                    ("/image_raw", "/camera1/image_raw"),
                    ("/image_raw/compressed", "/camera1/image_raw/compressed"),
                    ("/image_raw/theora", "/camera1/image_raw/theora"),
                    ("/camera_info", "/camera1/camera_info"),
                ],
            ),
            # 2つ目のカメラ (LifeCam)
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="camera2",  # ノード名をシンプルに
                parameters=[
                    {
                        "video_device": "/dev/video2",
                    }
                ],
                # 🟢 トピックが衝突しないように、名前の頭に /camera2 を強制付与する
                remappings=[
                    ("/image_raw", "/camera2/image_raw"),
                    ("/image_raw/compressed", "/camera2/image_raw/compressed"),
                    ("/image_raw/theora", "/camera2/image_raw/theora"),
                    ("/camera_info", "/camera2/camera_info"),
                ],
            ),
        ]
    )
