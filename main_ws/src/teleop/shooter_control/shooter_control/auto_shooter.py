import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class AutoShooter(Node):
    def __init__(self):
        super().__init__('auto_shooter')
        
        # Curve fitting parameters for 2nd order polynomial
        # target_rpm = A * (dist^2) + B * (dist) + C
        # These values must be calibrated on the real robot!
        self.declare_parameter('poly_a', 15.0)
        self.declare_parameter('poly_b', 100.0)
        self.declare_parameter('poly_c', 1500.0)
        
        self.declare_parameter('max_rpm', 7500.0)
        self.declare_parameter('min_rpm', 0.0)
        
        self.poly_a = self.get_parameter('poly_a').value
        self.poly_b = self.get_parameter('poly_b').value
        self.poly_c = self.get_parameter('poly_c').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.min_rpm = self.get_parameter('min_rpm').value

        # Subscribers and Publishers
        self.sub_dist = self.create_subscription(
            Float64, '/target/distance', self.distance_callback, 10)
            
        self.pub_cmd = self.create_publisher(Float64, '/shooter/cmd_auto', 10)
        
        self.get_logger().info("Auto Shooter Strategy Node started. Ready for curve fitting!")

    def distance_callback(self, msg):
        dist = msg.data
        
        if dist < 0.0:
            self.get_logger().warn("Negative distance received. Stopping shooter.")
            self.publish_rpm(0.0)
            return
            
        # --- The Ultimate Curve Fitting Logic ---
        # Calculate target RPM based on quadratic equation
        target_rpm = (self.poly_a * (dist ** 2)) + (self.poly_b * dist) + self.poly_c
        
        # Clamp to physical limits
        if target_rpm > self.max_rpm:
            target_rpm = self.max_rpm
        elif target_rpm < self.min_rpm:
            target_rpm = self.min_rpm
            
        self.publish_rpm(target_rpm)
        
    def publish_rpm(self, rpm):
        out_msg = Float64()
        out_msg.data = float(rpm)
        self.pub_cmd.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutoShooter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
