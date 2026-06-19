# Copyright 2026 Tatsukiyano
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math


class SignalInjector(Node):
    def __init__(self):
        super().__init__("signal_injector")
        self.publisher_ = self.create_publisher(TwistStamped, "/cmd_vel_ext", 10)

        self.declare_parameter("mode", "step")
        self.declare_parameter("amplitude", 1.0)
        self.declare_parameter("frequency", 1.0)
        self.declare_parameter("frequency_start", 0.1)
        self.declare_parameter("frequency_end", 10.0)
        self.declare_parameter("duration", 5.0)

        self.start_time = None
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("Signal Injector Waiting for non-zero clock (Gazebo play)...")

    def timer_callback(self):
        now = self.get_clock().now()

        if now.nanoseconds == 0:
            return

        if self.start_time is None:
            self.start_time = now
            self.get_logger().info("Simulation clock detected. Starting injection...")
            return

        elapsed = (now - self.start_time).nanoseconds / 1e9

        mode = self.get_parameter("mode").value
        amplitude = self.get_parameter("amplitude").value
        duration = self.get_parameter("duration").value

        if elapsed > duration:
            self.get_logger().info(f"Signal injection finished. Elapsed: {elapsed:.2f}s")
            self.stop_robot()
            # Professional exit: stop the timer and then shutdown
            self.timer.cancel()
            rclpy.shutdown()
            return

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "base_link"

        if mode == "step":
            msg.twist.linear.x = amplitude
        elif mode == "sine":
            frequency = self.get_parameter("frequency").value
            msg.twist.linear.x = amplitude * math.sin(2 * math.pi * frequency * elapsed)
        elif mode == "chirp":
            f0 = self.get_parameter("frequency_start").value
            f1 = self.get_parameter("frequency_end").value
            phase = 2 * math.pi * (f0 * elapsed + 0.5 * (f1 - f0) * (elapsed**2) / duration)
            msg.twist.linear.x = amplitude * math.sin(phase)

        self.publisher_.publish(msg)

    def stop_robot(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = SignalInjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as e:
        # If rclpy.shutdown() was called inside, spin might raise error
        if rclpy.ok():
            print(f"Error in spin: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
