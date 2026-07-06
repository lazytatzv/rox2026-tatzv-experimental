import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

class ShooterTeleop(Node):
    def __init__(self):
        super().__init__('shooter_teleop')
        
        # Parameters mapped from teleop_mux.yaml
        self.declare_parameter('joy_button_fire', 1)
        self.declare_parameter('joy_button_charge', 3)
        self.declare_parameter('topic_joy', 'joy')
        self.declare_parameter('topic_shooter_cmd', 'shooter/cmd_joy')
        
        self.btn_fire = self.get_parameter('joy_button_fire').value
        self.btn_charge = self.get_parameter('joy_button_charge').value
        topic_joy = self.get_parameter('topic_joy').value
        topic_cmd = self.get_parameter('topic_shooter_cmd').value

        self.sub_joy = self.create_subscription(
            Joy, topic_joy, self.joy_callback, 10)
        
        self.pub_cmd = self.create_publisher(Int16, topic_cmd, 10)
        
        self.get_logger().info("Shooter Teleop started")

    def joy_callback(self, msg):
        out_msg = Int16()
        
        # Simple logic: Charge button = slow speed, Fire button = max speed
        if len(msg.buttons) > max(self.btn_fire, self.btn_charge):
            if msg.buttons[self.btn_fire] == 1:
                out_msg.data = 255
            elif msg.buttons[self.btn_charge] == 1:
                out_msg.data = 100
            else:
                out_msg.data = 0
                
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
