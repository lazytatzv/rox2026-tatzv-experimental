# Copyright 2026 Tatsukiyano
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


class AutoAnalyzer(Node):
    def __init__(self):
        super().__init__("auto_analyzer")
        self.publisher_ = self.create_publisher(TwistStamped, "/cmd_vel_ext", 10)
        self.phase_pub_ = self.create_publisher(String, "/control_analysis/phase", 10)

        self.declare_parameter("report_name", "full_analysis_report")
        self.declare_parameter("chirp_duration", 15.0)
        self.declare_parameter("step_duration", 5.0)
        self.declare_parameter("step_amplitude", 2.0)
        self.declare_parameter("chirp_amplitude", 1.0)
        self.declare_parameter("rest_duration", 2.0)
        self.declare_parameter("frequency_start", 0.1)
        self.declare_parameter("frequency_end", 15.0)
        self.declare_parameter("prbs_duration", 10.0)
        self.declare_parameter("prbs_amplitude", 1.0)
        self.declare_parameter("prbs_hold_time", 0.05)

        self.start_time = None
        self.prbs_state = 0x7F
        self.prbs_current_val = 0.0
        self.prbs_last_switch = 0.0
        self.phase = "WAIT_FOR_CLOCK"
        self.timer = self.create_timer(0.01, self.loop)
        self.get_logger().info("Super Auto-Analyzer Ready. Waiting for Gazebo...")

    def publish_phase(self, label):
        msg = String()
        msg.data = label
        self.phase_pub_.publish(msg)
        self.get_logger().info(f"[Phase] {label}")

    def loop(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return

        if self.phase == "WAIT_FOR_CLOCK":
            self.start_time = now
            self.phase = "WAIT_FOR_RECORDER"
            self.get_logger().info("Clock received. Waiting 3s for rosbag...")
            return

        elapsed = (now - self.start_time).nanoseconds / 1e9

        if self.phase == "WAIT_FOR_RECORDER":
            if elapsed < 3.0:
                return
            self.start_time = now
            self.phase = "CHIRP"
            self.publish_phase("CHIRP_START")
            self.get_logger().info(">>> Phase 1: Chirp (Freq Response)")
            return

        elapsed = (now - self.start_time).nanoseconds / 1e9

        if self.phase == "CHIRP":
            duration = self.get_parameter("chirp_duration").value
            if elapsed < duration:
                f0 = self.get_parameter("frequency_start").value
                f1 = self.get_parameter("frequency_end").value
                amp = self.get_parameter("chirp_amplitude").value
                phase = 2 * math.pi * (f0 * elapsed + 0.5 * (f1 - f0) * (elapsed**2) / duration)
                self.publish_vel(amp * math.sin(phase))
            else:
                self.publish_phase("CHIRP_END")
                self.phase = "STEP_WAIT"
                self.mid_time = now
                self.get_logger().info(">>> Chirp finished. Resting...")

        elif self.phase == "STEP_WAIT":
            self.publish_vel(0.0)
            rest = self.get_parameter("rest_duration").value
            if (now - self.mid_time).nanoseconds / 1e9 > rest:
                self.phase = "STEP"
                self.mid_time = now
                self.publish_phase("STEP_START")
                self.get_logger().info(">>> Phase 2: Step Response")

        elif self.phase == "STEP":
            duration = self.get_parameter("step_duration").value
            amp = self.get_parameter("step_amplitude").value
            if (now - self.mid_time).nanoseconds / 1e9 < duration:
                self.publish_vel(amp)
            else:
                self.publish_phase("STEP_END")
                self.phase = "PRBS_WAIT"
                self.mid_time = now
                self.get_logger().info(">>> Step finished. Resting for PRBS...")

        elif self.phase == "PRBS_WAIT":
            self.publish_vel(0.0)
            rest = self.get_parameter("rest_duration").value
            if (now - self.mid_time).nanoseconds / 1e9 > rest:
                self.phase = "PRBS"
                self.mid_time = now
                self.prbs_last_switch = 0.0
                self.publish_phase("PRBS_START")
                self.get_logger().info(">>> Phase 3: PRBS (Pseudo-Random Binary Sequence)")

        elif self.phase == "PRBS":
            duration = self.get_parameter("prbs_duration").value
            amp = self.get_parameter("prbs_amplitude").value
            hold = self.get_parameter("prbs_hold_time").value
            t = (now - self.mid_time).nanoseconds / 1e9

            if t < duration:
                if t - self.prbs_last_switch >= hold:
                    # PRBS with x^7 + x^6 + 1
                    new_bit = ((self.prbs_state >> 6) ^ (self.prbs_state >> 5)) & 1
                    self.prbs_state = ((self.prbs_state << 1) | new_bit) & 0x7F
                    self.prbs_current_val = amp if new_bit else -amp
                    self.prbs_last_switch = t
                self.publish_vel(self.prbs_current_val)
            else:
                self.publish_phase("PRBS_END")
                self.publish_phase("DONE")
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


if __name__ == "__main__":
    main()
