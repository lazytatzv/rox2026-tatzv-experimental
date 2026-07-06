import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import Int16

class AutoStrategyNode(Node):
    def __init__(self):
        super().__init__('auto_strategy_node')
        
        # State machine states
        self.STATE_IDLE = 0
        self.STATE_NAVIGATING = 1
        self.STATE_ALIGNING = 2
        self.STATE_SHOOTING = 3
        
        self.current_state = self.STATE_IDLE
        
        # Action Client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribers
        self.sub_detections = self.create_subscription(
            AprilTagDetectionArray, '/detections', self.tag_callback, 10)
        
        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self.pub_shooter = self.create_publisher(Int16, '/shooter/cmd_auto', 10)
        
        # Target Tag ID
        self.target_tag_id = 0
        self.latest_tag_x = None
        
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("Auto Strategy Node started. Ready for mission.")
        
        # Start mission automatically for testing
        self.start_mission()

    def start_mission(self):
        self.get_logger().info("Mission Started: Navigating to waypoint...")
        self.current_state = self.STATE_NAVIGATING
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 2.0
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('Navigation completed! Switching to ALIGNING state.')
        self.current_state = self.STATE_ALIGNING

    def tag_callback(self, msg):
        for detection in msg.detections:
            if detection.id == self.target_tag_id:
                # Camera image width is 640. Center is 320.
                self.latest_tag_x = detection.centre.x
                return
        self.latest_tag_x = None

    def loop(self):
        if self.current_state == self.STATE_ALIGNING:
            if self.latest_tag_x is None:
                # Spin slowly to find the tag
                twist = Twist()
                twist.angular.z = 0.5
                self.pub_cmd_vel.publish(twist)
            else:
                # P-controller for alignment
                error = 320.0 - self.latest_tag_x
                if abs(error) < 10.0:
                    # Aligned
                    twist = Twist()
                    self.pub_cmd_vel.publish(twist)
                    self.get_logger().info("Tag perfectly aligned! SHOOTING!")
                    self.current_state = self.STATE_SHOOTING
                else:
                    twist = Twist()
                    twist.angular.z = error * 0.005 # Kp
                    self.pub_cmd_vel.publish(twist)
                    
        elif self.current_state == self.STATE_SHOOTING:
            # Send PWM to shooter
            shooter_msg = Int16()
            shooter_msg.data = 200
            self.pub_shooter.publish(shooter_msg)
            
            # Mission complete
            self.get_logger().info("Mission Complete!")
            self.current_state = self.STATE_IDLE
            
        elif self.current_state == self.STATE_IDLE:
            # Ensure motors are stopped
            shooter_msg = Int16()
            shooter_msg.data = 0
            self.pub_shooter.publish(shooter_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutoStrategyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
