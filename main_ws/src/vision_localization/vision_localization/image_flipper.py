import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge

class ImageFlipper(Node):
    def __init__(self):
        super().__init__('image_flipper')
        self.bridge = CvBridge()
        
        # Sub/Pub Image
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(
            Image, '/camera/image_flipped', 10)
            
        # Sub/Pub CameraInfo
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_callback, 10)
        self.info_pub = self.create_publisher(
            CameraInfo, '/camera/camera_info_flipped', 10)

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            flipped_img = cv2.flip(cv_img, 1) # Horizontal flip
            flipped_msg = self.bridge.cv2_to_imgmsg(flipped_img, encoding='bgr8')
            flipped_msg.header = msg.header
            self.image_pub.publish(flipped_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to flip image: {str(e)}')

    def info_callback(self, msg):
        # Mirror the horizontal principal point cx: cx_new = width - cx_old
        flipped_info = msg
        flipped_info.k[2] = float(msg.width - msg.k[2])
        flipped_info.p[2] = float(msg.width - msg.p[2])
        self.info_pub.publish(flipped_info)

def main():
    rclpy.init()
    node = ImageFlipper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
