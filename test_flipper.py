import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

rclpy.init()
msg = CameraInfo()
msg.width = 640
try:
    msg.k[2] = 320.0
    print("k is mutable")
except Exception as e:
    print(f"k error: {e}")

try:
    msg.p[2] = 320.0
    print("p is mutable")
except Exception as e:
    print(f"p error: {e}")
