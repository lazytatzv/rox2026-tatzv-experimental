import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import sys

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.saved = False

    def listener_callback(self, msg):
        if not self.saved:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv2.imwrite('/root/lazytatzv_ws/main_ws/camera_view.png', cv_image)
                self.get_logger().info('Image saved successfully to main_ws/camera_view.png')
                self.saved = True
            except Exception as e:
                self.get_logger().error(f'Failed to save image: {str(e)}')
            sys.exit(0)

def main():
    rclpy.init()
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
