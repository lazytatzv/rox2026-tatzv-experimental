# Copyright 2026 Tatsukiyano
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


class AutoAnalyzer(Node):
    """Full-sequence excitation: Chirp → rest → Step (fwd) → rest → Step (rev) → rest → PRBS."""

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
        # Whether to also inject a negative step for symmetry analysis
        self.declare_parameter("bidirectional_step", True)

        self.start_time = None
        self.mid_time = None

        # PRBS state (randomised each run)
        self._prbs_state = 0x7F
        self._prbs_current_val = 0.0
        self._prbs_last_switch = 0.0

        self.phase = "WAIT_FOR_CLOCK"
        self.timer = self.create_timer(0.01, self.loop)
        self.get_logger().info("Auto-Analyzer Ready. Waiting for Gazebo clock...")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def publish_phase(self, label: str):
        msg = String()
        msg.data = label
        self.phase_pub_.publish(msg)
        self.get_logger().info(f"[Phase] {label}")

    def publish_vel(self, vx: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(vx)
        self.publisher_.publish(msg)

    def _elapsed_mid(self, now) -> float:
        return (now - self.mid_time).nanoseconds / 1e9

    def _init_prbs(self, now):
        """Randomise PRBS seed based on PID and current clock for unbiased excitation."""
        raw = (os.getpid() ^ (now.nanoseconds & 0x7F)) & 0x7F
        seed = raw if raw != 0 else 0x5A
        self._prbs_state = seed
        self._prbs_current_val = 0.0
        self._prbs_last_switch = 0.0
        self.get_logger().info(f"  PRBS seed: 0x{seed:02X}")

    def _tick_prbs(self, t: float) -> float:
        """LFSR x^7 + x^6 + 1, returns ±amplitude."""
        hold = self.get_parameter("prbs_hold_time").value
        amp = self.get_parameter("prbs_amplitude").value
        if t - self._prbs_last_switch >= hold:
            new_bit = ((self._prbs_state >> 6) ^ (self._prbs_state >> 5)) & 1
            self._prbs_state = ((self._prbs_state << 1) | new_bit) & 0x7F
            self._prbs_current_val = amp if new_bit else -amp
            self._prbs_last_switch = t
        return self._prbs_current_val

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def loop(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return

        # ── WAIT_FOR_CLOCK ──────────────────────────────────────────
        if self.phase == "WAIT_FOR_CLOCK":
            self.start_time = now
            self.phase = "WAIT_FOR_RECORDER"
            self.get_logger().info("Clock received. Waiting 3 s for rosbag to start...")
            return

        elapsed_total = (now - self.start_time).nanoseconds / 1e9

        # ── WAIT_FOR_RECORDER ────────────────────────────────────────
        if self.phase == "WAIT_FOR_RECORDER":
            if elapsed_total < 3.0:
                return
            self.mid_time = now
            self.phase = "CHIRP"
            self.publish_phase("CHIRP_START")
            self.get_logger().info(">>> Phase 1: Chirp (frequency response)")
            return

        # ── CHIRP ────────────────────────────────────────────────────
        if self.phase == "CHIRP":
            duration = self.get_parameter("chirp_duration").value
            t = self._elapsed_mid(now)
            if t < duration:
                f0 = self.get_parameter("frequency_start").value
                f1 = self.get_parameter("frequency_end").value
                amp = self.get_parameter("chirp_amplitude").value
                phase_val = 2 * math.pi * (f0 * t + 0.5 * (f1 - f0) * (t**2) / duration)
                self.publish_vel(amp * math.sin(phase_val))
            else:
                self.publish_phase("CHIRP_END")
                self.publish_vel(0.0)
                self.phase = "STEP_WAIT"
                self.mid_time = now
                self.get_logger().info(">>> Chirp done. Resting...")
            return

        # ── STEP_WAIT (rest before positive step) ────────────────────
        if self.phase == "STEP_WAIT":
            self.publish_vel(0.0)
            if self._elapsed_mid(now) > self.get_parameter("rest_duration").value:
                self.phase = "STEP_FWD"
                self.mid_time = now
                self.publish_phase("STEP_START")
                self.get_logger().info(">>> Phase 2a: Positive Step Response")
            return

        # ── STEP_FWD ─────────────────────────────────────────────────
        if self.phase == "STEP_FWD":
            duration = self.get_parameter("step_duration").value
            amp = self.get_parameter("step_amplitude").value
            if self._elapsed_mid(now) < duration:
                self.publish_vel(amp)
            else:
                self.publish_phase("STEP_END")
                self.publish_vel(0.0)
                if self.get_parameter("bidirectional_step").value:
                    self.phase = "STEP_REV_WAIT"
                    self.mid_time = now
                    self.get_logger().info(">>> Positive step done. Resting for reverse step...")
                else:
                    self.phase = "PRBS_WAIT"
                    self.mid_time = now
                    self.get_logger().info(">>> Step done. Resting for PRBS...")
            return

        # ── STEP_REV_WAIT (rest before negative step) ─────────────────
        if self.phase == "STEP_REV_WAIT":
            self.publish_vel(0.0)
            if self._elapsed_mid(now) > self.get_parameter("rest_duration").value:
                self.phase = "STEP_REV"
                self.mid_time = now
                self.publish_phase("STEP_NEG_START")
                self.get_logger().info(">>> Phase 2b: Negative Step Response")
            return

        # ── STEP_REV ─────────────────────────────────────────────────
        if self.phase == "STEP_REV":
            duration = self.get_parameter("step_duration").value
            amp = self.get_parameter("step_amplitude").value
            if self._elapsed_mid(now) < duration:
                self.publish_vel(-amp)
            else:
                self.publish_phase("STEP_NEG_END")
                self.publish_vel(0.0)
                self.phase = "PRBS_WAIT"
                self.mid_time = now
                self.get_logger().info(">>> Reverse step done. Resting for PRBS...")
            return

        # ── PRBS_WAIT ────────────────────────────────────────────────
        if self.phase == "PRBS_WAIT":
            self.publish_vel(0.0)
            if self._elapsed_mid(now) > self.get_parameter("rest_duration").value:
                self.phase = "PRBS"
                self._init_prbs(now)
                self.mid_time = now
                self.publish_phase("PRBS_START")
                self.get_logger().info(">>> Phase 3: PRBS (Pseudo-Random Binary Sequence)")
            return

        # ── PRBS ─────────────────────────────────────────────────────
        if self.phase == "PRBS":
            duration = self.get_parameter("prbs_duration").value
            t = self._elapsed_mid(now)
            if t < duration:
                self.publish_vel(self._tick_prbs(t))
            else:
                self.publish_phase("PRBS_END")
                self.publish_phase("DONE")
                self.publish_vel(0.0)
                self.phase = "FINISH"
                self.get_logger().info(">>> All tests finished!")
                raise SystemExit


def main():
    rclpy.init()
    node = AutoAnalyzer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
