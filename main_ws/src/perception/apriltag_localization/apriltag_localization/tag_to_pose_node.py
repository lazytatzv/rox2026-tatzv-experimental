#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import numpy as np
import math
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped

# Known positions of AprilTags on the RoboMaster Field (Example absolute coordinates)
# X, Y, Z, Roll, Pitch, Yaw (in radians)
TAG_POSES = {
    0: (4.0, 2.5, 0.5, 0.0, 0.0, 0.0), # Example: Center of the field
    1: (1.0, 1.0, 0.5, 0.0, 0.0, 0.0), # Example: Corner tag
    # TODO: Populate with exact coordinates from the CAD/Rulebook
}

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk
    
    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q

class TagLocalizationNode(Node):
    def __init__(self):
        super().__init__('tag_localization_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/apriltag_pose', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("AprilTag Localization Node Started.")

    def timer_callback(self):
        # We listen to tf for any of the tags
        for tag_id, tag_pose in TAG_POSES.items():
            tag_frame = f"tag_{tag_id}"
            try:
                # We want to find where base_footprint is relative to the TAG
                # So we look up the transform FROM tag_X TO base_footprint
                t = self.tf_buffer.lookup_transform(tag_frame, 'base_footprint', rclpy.time.Time())
                
                # We know where the tag is in the map frame (TAG_POSES)
                # We can calculate where base_footprint is in the map frame!
                
                p_base_in_tag = PoseStamped()
                p_base_in_tag.header.frame_id = tag_frame
                p_base_in_tag.pose.position.x = t.transform.translation.x
                p_base_in_tag.pose.position.y = t.transform.translation.y
                p_base_in_tag.pose.position.z = t.transform.translation.z
                p_base_in_tag.pose.orientation = t.transform.rotation
                
                t_map_tag = TransformStamped()
                t_map_tag.header.frame_id = 'map'
                t_map_tag.child_frame_id = tag_frame
                t_map_tag.transform.translation.x = tag_pose[0]
                t_map_tag.transform.translation.y = tag_pose[1]
                t_map_tag.transform.translation.z = tag_pose[2]
                q = quaternion_from_euler(tag_pose[3], tag_pose[4], tag_pose[5])
                t_map_tag.transform.rotation.x = q[0]
                t_map_tag.transform.rotation.y = q[1]
                t_map_tag.transform.rotation.z = q[2]
                t_map_tag.transform.rotation.w = q[3]
                
                # Now transform!
                p_base_in_map = do_transform_pose(p_base_in_tag.pose, t_map_tag)
                
                # Publish the absolute pose!
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'map'
                pose_msg.pose.pose = p_base_in_map
                
                # Dynamic covariance based on distance
                dist = math.sqrt(t.transform.translation.x**2 + t.transform.translation.y**2 + t.transform.translation.z**2)
                cov_val = max(0.01, dist * 0.05) # Covariance scales with distance
                
                cov = np.zeros((6, 6))
                np.fill_diagonal(cov, [cov_val, cov_val, 0.05, 0.1, 0.1, cov_val * 2])
                pose_msg.pose.covariance = cov.flatten().tolist()
                
                self.pose_pub.publish(pose_msg)
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

def main(args=None):
    rclpy.init(args=args)
    node = TagLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
