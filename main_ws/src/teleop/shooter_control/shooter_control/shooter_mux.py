import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ShooterMux(Node):
    def __init__(self):
        super().__init__('shooter_mux')
        
        # Parameters
        self.declare_parameter('teleop_timeout', 0.5)
        self.declare_parameter('topic_cmd_joy', '/shooter/cmd_joy')
        self.declare_parameter('topic_cmd_auto', '/shooter/cmd_auto')
        self.declare_parameter('topic_cmd_muxed', '/shooter/cmd_muxed')
        
        self.timeout = self.get_parameter('teleop_timeout').value
        t_joy = self.get_parameter('topic_cmd_joy').value
        t_auto = self.get_parameter('topic_cmd_auto').value
        t_muxed = self.get_parameter('topic_cmd_muxed').value

        # State
        self.latest_teleop = 0.0
        self.latest_auto = 0.0
        self.last_teleop_time = self.get_clock().now()

        # Subscribers
        self.sub_teleop = self.create_subscription(
            Float64, t_joy, self.teleop_callback, 10)
        self.sub_auto = self.create_subscription(
            Float64, t_auto, self.auto_callback, 10)

        # Publisher
        self.pub_muxed = self.create_publisher(Float64, t_muxed, 10)

        # Timer (50Hz to match motor command rate)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info(f"Shooter Mux started with teleop timeout: {self.timeout}s")

    def teleop_callback(self, msg):
        self.latest_teleop = msg.data
        self.last_teleop_time = self.get_clock().now()

    def auto_callback(self, msg):
        self.latest_auto = msg.data

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_teleop_time).nanoseconds / 1e9

        out_msg = Float64()
        
        # If teleop is active and hasn't timed out, use it.
        # Otherwise, fall back to auto.
        if elapsed <= self.timeout:
            out_msg.data = self.latest_teleop
        else:
            out_msg.data = self.latest_auto
            
        self.pub_muxed.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ShooterMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
