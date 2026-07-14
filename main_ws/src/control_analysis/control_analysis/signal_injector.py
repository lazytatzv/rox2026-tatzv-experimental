# Copyright 2026 Tatsukiyano
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math
import os


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
        # PRBS-specific parameters
        self.declare_parameter("prbs_hold_time", 0.05)  # seconds per bit
        self.declare_parameter("prbs_seed", -1)  # -1 = randomize each run

        self.start_time = None
        self._prbs_state = None
        self._prbs_val = 0.0
        self._prbs_last_switch = 0.0

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("Signal Injector: waiting for non-zero clock (Gazebo play)...")

    # ------------------------------------------------------------------
    # PRBS helpers
    # ------------------------------------------------------------------
    def _init_prbs(self):
        seed_param = self.get_parameter("prbs_seed").value
        if seed_param < 0:
            # Use process ID + nanoseconds for a different seed every run
            seed = (os.getpid() ^ (self.get_clock().now().nanoseconds & 0x7F)) & 0x7F
            seed = seed if seed != 0 else 0x7F  # avoid all-zero state
        else:
            seed = int(seed_param) & 0x7F
            seed = seed if seed != 0 else 0x7F
        self._prbs_state = seed
        self._prbs_val = 0.0
        self._prbs_last_switch = 0.0
        self.get_logger().info(f"  PRBS seed: 0x{seed:02X}")

    def _tick_prbs(self, elapsed: float) -> float:
        """Advance the PRBS LFSR (x^7 + x^6 + 1) and return current value."""
        hold = self.get_parameter("prbs_hold_time").value
        amp = self.get_parameter("amplitude").value
        if elapsed - self._prbs_last_switch >= hold:
            new_bit = ((self._prbs_state >> 6) ^ (self._prbs_state >> 5)) & 1
            self._prbs_state = ((self._prbs_state << 1) | new_bit) & 0x7F
            self._prbs_val = amp if new_bit else -amp
            self._prbs_last_switch = elapsed
        return self._prbs_val

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------
    def timer_callback(self):
        now = self.get_clock().now()

        if now.nanoseconds == 0:
            return

        if self.start_time is None:
            mode = self.get_parameter("mode").value
            amp = self.get_parameter("amplitude").value
            duration = self.get_parameter("duration").value
            self.get_logger().info(
                f"Simulation clock detected. Starting injection. "
                f"mode={mode}, amplitude={amp:.2f}, duration={duration:.1f}s"
            )
            self.start_time = now
            if mode == "prbs":
                self._init_prbs()
            return

        elapsed = (now - self.start_time).nanoseconds / 1e9
        mode = self.get_parameter("mode").value
        amplitude = self.get_parameter("amplitude").value
        duration = self.get_parameter("duration").value

        if elapsed > duration:
            self.get_logger().info(f"Signal injection finished. Elapsed: {elapsed:.2f}s")
            self.stop_robot()
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
        elif mode == "prbs":
            msg.twist.linear.x = self._tick_prbs(elapsed)
        else:
            self.get_logger().warn(
                f"Unknown mode '{mode}'. Valid: step, sine, chirp, prbs. Sending zero."
            )
            msg.twist.linear.x = 0.0

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
        if rclpy.ok():
            print(f"Error in spin: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
