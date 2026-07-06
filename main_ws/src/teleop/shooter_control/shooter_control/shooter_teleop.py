import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class ShooterTeleop(Node):
    def __init__(self):
        super().__init__('shooter_teleop')
        
        # Parameters mapped from PR #48 style (but using RPM)
        self.declare_parameter('enable_button', 6)  # L2 usually
        self.declare_parameter('high_button', 2)
        self.declare_parameter('medium_button', 1)
        self.declare_parameter('low_button', 0)
        
        self.declare_parameter('high_rpm', 5000.0)
        self.declare_parameter('medium_rpm', 3000.0)
        self.declare_parameter('low_rpm', 1000.0)
        self.declare_parameter('stop_rpm', 0.0)
        
        self.declare_parameter('topic_joy', 'joy')
        self.declare_parameter('topic_shooter_cmd', 'shooter/cmd_joy')
        
        self.enable_btn = self.get_parameter('enable_button').value
        self.high_btn = self.get_parameter('high_button').value
        self.med_btn = self.get_parameter('medium_button').value
        self.low_btn = self.get_parameter('low_button').value

        self.high_rpm = self.get_parameter('high_rpm').value
        self.med_rpm = self.get_parameter('medium_rpm').value
        self.low_rpm = self.get_parameter('low_rpm').value
        self.stop_rpm = self.get_parameter('stop_rpm').value

        topic_joy = self.get_parameter('topic_joy').value
        topic_cmd = self.get_parameter('topic_shooter_cmd').value

        self.sub_joy = self.create_subscription(Joy, topic_joy, self.joy_callback, 10)
        self.pub_cmd = self.create_publisher(Float64, topic_cmd, 10)
        
        self.last_target = self.stop_rpm

        self.get_logger().info("Shooter Teleop (RPM Control) started with Deadman Switch")

    def joy_callback(self, msg):
        out_msg = Float64()
        target_rpm = self.last_target
        
        buttons = msg.buttons
        
        # SAFETY: Deadman Switch Check (e.g. L2)
        if len(buttons) <= self.enable_btn or buttons[self.enable_btn] == 0:
            target_rpm = self.stop_rpm  # Mandatory stop if L2 is released!
        else:
            # Determine RPM based on button pressed
            if len(buttons) > self.high_btn and buttons[self.high_btn] == 1:
                target_rpm = self.high_rpm
            elif len(buttons) > self.med_btn and buttons[self.med_btn] == 1:
                target_rpm = self.med_rpm
            elif len(buttons) > self.low_btn and buttons[self.low_btn] == 1:
                target_rpm = self.low_rpm

        self.last_target = target_rpm
        out_msg.data = target_rpm
        self.pub_cmd.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ShooterTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
