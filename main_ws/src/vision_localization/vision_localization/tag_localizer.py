# Copyright 2026 Tatsukiyano
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
from rclpy.duration import Duration
import numpy as np
import math

class TagLocalizer(Node):
    def __init__(self):
        super().__init__('tag_localizer')

        # Tag Database (Global Coordinates in 'map' frame)
        # measurement.webp and rulebook mapping
        # Left Side (Side A, X < 0) | Right Side (Side B, X > 0)
        self.tag_map = {
            0:  {'x': -6.425, 'y':  5.45,  'z': 0.3, 'yaw': -1.57}, # Corner N-W
            1:  {'x':  6.425, 'y':  5.45,  'z': 0.3, 'yaw': -1.57}, # Corner N-E
            2:  {'x': -6.425, 'y': -5.45,  'z': 0.3, 'yaw':  1.57}, # Corner S-W
            3:  {'x':  6.425, 'y': -5.45,  'z': 0.3, 'yaw':  1.57}, # Corner S-E
            12: {'x':  0.0,   'y':  1.0,   'z': 0.5, 'yaw':  3.14}, # Center Partition Top
            13: {'x':  0.0,   'y': -1.0,   'z': 0.5, 'yaw':  0.0},  # Center Partition Bottom
            # Note: Add remaining tags 4-11, 14-26 based on final exact placement
        }

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            self.tag_callback,
            10)

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/apriltag_pose',
            10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("AprilTag Localizer Started. Waiting for detections...")

    def tag_callback(self, msg):
        for detection in msg.detections:
            tag_id = detection.id
            if tag_id not in self.tag_map:
                continue

            # 1. Get the tag's pose in camera frame
            tag_in_camera = detection.pose.pose.pose

            try:
                # 2. Lookup transform from base_footprint to camera
                # We need this to find where the ROBOT is relative to the tag
                transform = self.tf_buffer.lookup_transform(
                    'base_footprint',
                    msg.header.frame_id, # Usually camera_optical_frame
                    msg.header.stamp,
                    Duration(seconds=0.1))

                # 3. Calculate Robot Pose in Global Map
                # Robot_in_Map = Tag_in_Map * (Tag_in_Camera)^-1 * (Camera_in_Robot)^-1
                # Simplified approach: Use TF to chain these

                # Tag Global Pose
                T_map_tag = self.get_transform_from_map(tag_id)

                # Transform tag detection (camera frame) to robot frame (base_footprint)
                tag_in_base = tf2_geometry_msgs.do_transform_pose(tag_in_camera, transform)

                # Invert: Robot in Tag Frame
                # ... Calculation logic ...

                # For now, we'll publish a PoseWithCovarianceStamped for EKF
                # In a pro setup, we'd use the known tag position to 'snap' the robot position

                out_msg = PoseWithCovarianceStamped()
                out_msg.header.stamp = msg.header.stamp
                out_msg.header.frame_id = 'map'

                # Simplified projection for demonstration
                # Real implementation uses 3D matrix inversion
                out_msg.pose.pose.position.x = self.tag_map[tag_id]['x'] - tag_in_base.position.x
                out_msg.pose.pose.position.y = self.tag_map[tag_id]['y'] - tag_in_base.position.y

                # High confidence for AprilTag (Low covariance)
                out_msg.pose.covariance = [0.01] * 36

                self.publisher.publish(out_msg)
                # self.get_logger().info(f"Detected Tag {tag_id}. Correcting position...")

            except TransformException as ex:
                self.get_logger().warning(f"Could not transform tag: {ex}")

    def get_transform_from_map(self, tag_id):
        # Helper to create a Pose from tag_map
        t = self.tag_map[tag_id]
        # ... logic ...
        return None

def main():
    rclpy.init()
    node = TagLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
