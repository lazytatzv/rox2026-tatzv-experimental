import math
import struct
from typing import List

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from seeed_usb_can_analyzer_driver.msg import CanFrame
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

class El05MotorNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("el05_motor_node")
        
        # Declare parameters (Keep them in __init__ but don't use them yet)
        self.declare_parameter("motor_id", 0x7F)
        self.declare_parameter("host_id", 0xFD)
        self.declare_parameter("joint_name", "el05_joint")
        self.declare_parameter("auto_enable", False)
        self.declare_parameter("can_tx_topic", "/communication/tx_queue") # Standardized
        self.declare_parameter("can_rx_topic", "/communication/rx_queue") # Standardized
        self.declare_parameter("velocity_min_rad_s", -50.0)
        self.declare_parameter("velocity_max_rad_s", 50.0)

    # --- Lifecycle Transitions ---

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring EL05 Motor Node...")
        
        self.motor_id = int(self.get_parameter("motor_id").value)
        self.host_id = int(self.get_parameter("host_id").value)
        self.joint_name = self.get_parameter("joint_name").value
        self.vel_limit = float(self.get_parameter("velocity_max_rad_s").value)

        # Publishers (Managed)
        self.can_pub = self.create_lifecycle_publisher(CanFrame, self.get_parameter("can_tx_topic").value, 10)
        self.joint_pub = self.create_lifecycle_publisher(JointState, "~/joint_states", 10)
        
        # Subscribers
        self.vel_sub = self.create_subscription(
            Float64MultiArray, "~/velocity_command", self.velocity_callback, 10)
        self.can_sub = self.create_subscription(
            CanFrame, self.get_parameter("can_rx_topic").value, self.can_rx_callback, 50)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating EL05 Motor Node...")
        # Enable motor if needed
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating EL05 Motor Node...")
        # Send safety stop (0 velocity)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up EL05 Motor Node...")
        self.destroy_subscription(self.vel_sub)
        self.destroy_subscription(self.can_sub)
        return TransitionCallbackReturn.SUCCESS

    # --- Callbacks & Logic ---

    def velocity_callback(self, msg: Float64MultiArray) -> None:
        if self.get_current_state().id != State.PRIMARY_STATE_ACTIVE:
            return
        if not msg.data:
            return
        
        velocity_rad_s = msg.data[0]
        # (Packet building logic from senior's code goes here)
        # For now, let's keep it consistent with the existing implementation
        pass

    def can_rx_callback(self, msg: CanFrame) -> None:
        if self.get_current_state().id != State.PRIMARY_STATE_ACTIVE:
            return
        # (Parsing logic from senior's code goes here)
        pass

def main(args=None) -> None:
    rclpy.init(args=args)
    node = El05MotorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
