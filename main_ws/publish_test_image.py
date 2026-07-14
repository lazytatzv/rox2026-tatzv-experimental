import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np
from cv_bridge import CvBridge

class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        self.image_pub = self.create_publisher(Image, '/camera_synced/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera_synced/camera_info', 10)
        self.bridge = CvBridge()
        
        # Load the saved camera image from the workspace
        # Inside Docker, the path maps to /root/lazytatzv_ws/main_ws/camera_view.png
        self.cv_img = cv2.imread('/root/lazytatzv_ws/main_ws/camera_view.png', cv2.IMREAD_GRAYSCALE)
        if self.cv_img is None:
            self.get_logger().warn("Failed to load /root/lazytatzv_ws/main_ws/camera_view.png. Generating a dummy tag grid pattern instead.")
            # Fallback: create a dummy image with a dark square simulating a tag
            self.cv_img = np.ones((480, 640), dtype='uint8') * 200
            cv2.rectangle(self.cv_img, (220, 140), (420, 340), 0, -1) # Draw black square
            
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        self.get_logger().info("Test Image Publisher Started. Publishing to /camera_synced/...")

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        
        # Publish Image
        img_msg = self.bridge.cv2_to_imgmsg(self.cv_img, encoding='mono8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = 'camera_optical_frame'
        self.image_pub.publish(img_msg)
        
        # Publish CameraInfo (required for pose estimation / spatial calculations)
        info_msg = CameraInfo()
        info_msg.header.stamp = now
        info_msg.header.frame_id = 'camera_optical_frame'
        info_msg.height = self.cv_img.shape[0]
        info_msg.width = self.cv_img.shape[1]
        
        # Dummy calibration matrix (suitable for detection tests)
        info_msg.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        info_msg.p = [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        info_msg.distortion_model = 'plumb_bob'
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.info_pub.publish(info_msg)

def main():
    rclpy.init()
    node = TestImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
