import rclpy
from geometry_msgs.msg import Twist
import time

rclpy.init()
node = rclpy.create_node('test_pub')
pub = node.create_publisher(Twist, '/cmd_vel', 10)
msg = Twist()
msg.linear.x = 0.5
while rclpy.ok():
    pub.publish(msg)
    time.sleep(0.1)
