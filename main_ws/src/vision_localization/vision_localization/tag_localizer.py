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
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped

class TagLocalizer(Node):
    def __init__(self):
        super().__init__("tag_localizer")

        # Tag Database (Global Coordinates in 'map' frame)
        # measurement.webp and rulebook mapping
        # Left Side (Side A, X < 0) | Right Side (Side B, X > 0)
        self.tag_map = {
            0: {"x": -6.425, "y": 5.45, "z": 0.3, "yaw": -1.57},  # Corner N-W
            1: {"x": 6.425, "y": 5.45, "z": 0.3, "yaw": -1.57},  # Corner N-E
            2: {"x": -6.425, "y": -5.45, "z": 0.3, "yaw": 1.57},  # Corner S-W
            3: {"x": 6.425, "y": -5.45, "z": 0.3, "yaw": 1.57},  # Corner S-E
            12: {"x": 0.0, "y": 1.0, "z": 0.5, "yaw": 3.14},  # Center Partition Top
            13: {"x": 0.0, "y": -1.0, "z": 0.5, "yaw": 0.0},  # Center Partition Bottom
            # Note: Add remaining tags 4-11, 14-26 based on final exact placement
        }

        self.subscription = self.create_subscription(
            AprilTagDetectionArray, "/apriltag_detections", self.tag_callback, 10
        )

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, "/apriltag_pose", 10)

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
                    "base_footprint",
                    msg.header.frame_id,  # Usually camera_optical_frame
                    msg.header.stamp,
                    Duration(seconds=0.1),
                )

                # 3. Calculate Robot Pose in Global Map
                # Robot_in_Map = Tag_in_Map * (Tag_in_Camera)^-1 * (Camera_in_Robot)^-1
                
                # Tag Global Pose (from our dictionary)
                tag_global = self.tag_map[tag_id]
                
                # We want to find the transform FROM tag_frame TO base_footprint
                # The user's lookup_transform gives base_footprint -> camera_optical_frame
                # Let's get the full transform directly using tf2
                t_tag_base = self.tf_buffer.lookup_transform(
                    f"tag_{tag_id}",  # Target frame
                    "base_footprint", # Source frame
                    msg.header.stamp,
                    Duration(seconds=0.1),
                )
                
                # Construct PoseStamped for base_footprint in tag_frame
                p_base_in_tag = PoseStamped()
                p_base_in_tag.header.frame_id = f"tag_{tag_id}"
                p_base_in_tag.pose.position.x = t_tag_base.transform.translation.x
                p_base_in_tag.pose.position.y = t_tag_base.transform.translation.y
                p_base_in_tag.pose.position.z = t_tag_base.transform.translation.z
                p_base_in_tag.pose.orientation = t_tag_base.transform.rotation
                
                # Create TransformStamped for map -> tag_frame
                t_map_tag = TransformStamped()
                t_map_tag.header.frame_id = "map"
                t_map_tag.child_frame_id = f"tag_{tag_id}"
                t_map_tag.transform.translation.x = tag_global["x"]
                t_map_tag.transform.translation.y = tag_global["y"]
                t_map_tag.transform.translation.z = tag_global["z"]
                
                # Convert Euler Yaw to Quaternion
                yaw = tag_global["yaw"]
                t_map_tag.transform.rotation.z = math.sin(yaw / 2.0)
                t_map_tag.transform.rotation.w = math.cos(yaw / 2.0)
                
                # Transform base_footprint pose to map frame
                p_base_in_map = do_transform_pose(p_base_in_tag.pose, t_map_tag)

                out_msg = PoseWithCovarianceStamped()
                out_msg.header.stamp = msg.header.stamp
                out_msg.header.frame_id = "map"

                out_msg.pose.pose = p_base_in_map

                # Dynamic covariance based on distance
                dist = math.sqrt(t_tag_base.transform.translation.x**2 + t_tag_base.transform.translation.y**2)
                cov_val = max(0.01, dist * 0.05) # Uncertainty grows with distance
                cov = np.zeros((6, 6))
                np.fill_diagonal(cov, [cov_val, cov_val, 0.05, 0.1, 0.1, cov_val * 2])
                out_msg.pose.covariance = cov.flatten().tolist()

                self.publisher.publish(out_msg)

            except TransformException as ex:
                self.get_logger().warning(f"Could not transform tag: {ex}")

    def get_transform_from_map(self, tag_id):
        # Deprecated
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


if __name__ == "__main__":
    main()
