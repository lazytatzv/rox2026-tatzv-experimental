import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray

class DetectionChecker(Node):
    def __init__(self):
        super().__init__('detection_checker')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.listener_callback,
            10)
        self.get_logger().info("Listening to /detections... Waiting for AprilTag detections.")

    def listener_callback(self, msg):
        if not msg.detections:
            return
        
        detected_ids = [d.id for d in msg.detections]
        self.get_logger().info(f"Detected {len(detected_ids)} tags: {detected_ids}")
        for d in msg.detections:
            pos = d.pose.pose.pose.position
            self.get_logger().info(f"  - Tag {d.id} Pose: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}")

def main():
    rclpy.init()
    node = DetectionChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
