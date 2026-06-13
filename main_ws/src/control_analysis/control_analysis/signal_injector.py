import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math
import time

class SignalInjector(Node):
    def __init__(self):
        super().__init__('signal_injector')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel_ext', 10)
        
        self.declare_parameter('mode', 'step')  # 'step', 'sine'
        self.declare_parameter('amplitude', 1.0)
        self.declare_parameter('frequency', 1.0) # For sine mode (Hz)
        self.declare_parameter('duration', 5.0)

        self.start_time = self.get_clock().now()
        self.timer = self.create_wall_timer(0.01, self.timer_callback)
        self.get_logger().info("Signal Injector Started. Waiting for signals...")

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        mode = self.get_parameter('mode').value
        amplitude = self.get_parameter('amplitude').value
        duration = self.get_parameter('duration').value
        
        if elapsed > duration:
            self.get_logger().info("Signal injection finished.")
            self.stop_robot()
            raise SystemExit

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "base_footprint"

        if mode == 'step':
            msg.twist.linear.x = amplitude
        elif mode == 'sine':
            frequency = self.get_parameter('frequency').value
            msg.twist.linear.x = amplitude * math.sin(2 * math.pi * frequency * elapsed)
        
        self.publisher_.publish(msg)

    def stop_robot(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = SignalInjector()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
