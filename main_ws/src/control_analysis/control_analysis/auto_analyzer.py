# Copyright 2026 Tatsukiyano
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math

class AutoAnalyzer(Node):
    def __init__(self):
        super().__init__('auto_analyzer')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel_ext', 10)

        self.declare_parameter('report_name', 'full_analysis_report')
        self.declare_parameter('chirp_duration', 15.0)
        self.declare_parameter('step_duration', 5.0)
        self.declare_parameter('frequency_start', 0.1)
        self.declare_parameter('frequency_end', 15.0)

        self.start_time = None
        self.phase = "WAIT_FOR_CLOCK"
        self.timer = self.create_timer(0.01, self.loop)
        self.get_logger().info("Super Auto-Analyzer Ready. Waiting for Gazebo...")

    def loop(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0: return

        if self.phase == "WAIT_FOR_CLOCK":
            self.start_time = now
            self.phase = "CHIRP"
            self.get_logger().info(">>> Phase 1: Chirp (Freq Response)")
            return

        elapsed = (now - self.start_time).nanoseconds / 1e9

        if self.phase == "CHIRP":
            d = self.get_parameter('chirp_duration').value
            if elapsed < d:
                f0 = self.get_parameter('frequency_start').value
                f1 = self.get_parameter('frequency_end').value
                p = 2 * math.pi * (f0 * elapsed + 0.5 * (f1 - f0) * (elapsed**2) / d)
                self.publish_vel(1.0 * math.sin(p))
            else:
                self.phase = "STEP_WAIT"
                self.mid_time = now
                self.get_logger().info(">>> Chirp finished. Resting...")

        elif self.phase == "STEP_WAIT":
            # CRITICAL: Keep publishing 0.0 to define the baseline in logs
            self.publish_vel(0.0)
            if (now - self.mid_time).nanoseconds / 1e9 > 2.0:
                self.phase = "STEP"
                self.mid_time = now
                self.get_logger().info(">>> Phase 2: Step Response")

        elif self.phase == "STEP":
            d = self.get_parameter('step_duration').value
            if (now - self.mid_time).nanoseconds / 1e9 < d:
                self.publish_vel(2.0)
            else:
                self.phase = "FINISH"
                self.get_logger().info(">>> All tests finished!")
                self.publish_vel(0.0)
                raise SystemExit

    def publish_vel(self, vx):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(vx)
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = AutoAnalyzer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
