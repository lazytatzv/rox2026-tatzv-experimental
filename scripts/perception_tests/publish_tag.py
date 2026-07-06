import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys

class TagPublisher(Node):
    def __init__(self):
        super().__init__('tag_publisher')
        self.img_pub = self.create_publisher(Image, '/camera_synced/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera_synced/camera_info', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        
        import urllib.request
        url = "https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag16h5/tag16_05_00000.png"
        resp = urllib.request.urlopen(url)
        img_array = np.asarray(bytearray(resp.read()), dtype=np.uint8)
        tag_img_small = cv2.imdecode(img_array, cv2.IMREAD_GRAYSCALE)
        
        # tag16h5 images from that repo are 7x7 pixels (including the 1px white border) or something similar.
        # We need to scale it up massively without blurring so the detector can find it.
        tag_img = cv2.resize(tag_img_small, (200, 200), interpolation=cv2.INTER_NEAREST)
        
        self.img = np.ones((480, 640), dtype=np.uint8) * 255
        self.img[140:340, 220:420] = tag_img
        
    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        
        img_msg = self.bridge.cv2_to_imgmsg(self.img, encoding="mono8")
        img_msg.header.stamp = now
        img_msg.header.frame_id = "camera_link_optical"
        
        info_msg = CameraInfo()
        info_msg.header.stamp = now
        info_msg.header.frame_id = "camera_link_optical"
        info_msg.width = 640
        info_msg.height = 480
        info_msg.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        info_msg.p = [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        self.img_pub.publish(img_msg)
        self.info_pub.publish(info_msg)

def main():
    rclpy.init()
    node = TagPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
