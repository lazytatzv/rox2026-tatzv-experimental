import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import message_filters

class ImageFlipper(Node):
    def __init__(self):
        super().__init__('image_flipper')
        self.bridge = CvBridge()
        
        self.image_pub = self.create_publisher(Image, '/camera_flipped/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera_flipped/camera_info', 10)
            
        # Use message_filters to synchronize Image and CameraInfo
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/camera_info')
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.info_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)

    def sync_callback(self, img_msg, info_msg):
        try:
            # Convert directly to mono8 (grayscale) to save bandwidth and CPU
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')
            # Do NOT flip the image, Gazebo Sim already provides correct textures!
            flipped_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='mono8')
            flipped_msg.header = img_msg.header
            self.image_pub.publish(flipped_msg)
            
            # Pass camera_info through directly with matching timestamp
            flipped_info = info_msg
            flipped_info.header.stamp = img_msg.header.stamp
            self.info_pub.publish(flipped_info)
        except Exception as e:
            self.get_logger().error(f'Failed to flip image: {str(e)}')

def main():
    rclpy.init()
    node = ImageFlipper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
