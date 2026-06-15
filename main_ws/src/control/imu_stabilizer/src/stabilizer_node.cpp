// Copyright 2026 Tatsukiyano
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "control_toolbox/pid.hpp"

using namespace std::chrono_literals;

class HeadingStabilizer : public rclcpp::Node {
 public:
  HeadingStabilizer() : Node("heading_stabilizer"),
    // --- EXPLICIT INITIALIZATION TO AVOID DEPRECATED CONSTRUCTORS ---
    pid_heading_(0.0, 0.0, 0.0, 1.0, -1.0, control_toolbox::AntiWindupStrategy()),
    pid_rate_(0.0, 0.0, 0.0, 0.5, -0.5, control_toolbox::AntiWindupStrategy())
  {
    declare_parameters();
    
    // Configure Anti-Windup Strategy
    control_toolbox::AntiWindupStrategy aw_strat;
    aw_strat.type = control_toolbox::AntiWindupStrategy::CONDITIONAL_INTEGRATION;
    
    // Now initialize with specific gains
    aw_strat.i_max = 1.0; aw_strat.i_min = -1.0;
    pid_heading_.initialize(3.0, 0.5, 0.0, 1.0, -1.0, aw_strat);
    
    aw_strat.i_max = 0.5; aw_strat.i_min = -0.5;
    pid_rate_.initialize(0.5, 0.0, 0.05, 0.5, -0.5, aw_strat);

    sub_cmd_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_in", 10, std::bind(&HeadingStabilizer::cmd_callback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10, std::bind(&HeadingStabilizer::odom_callback, this, std::placeholders::_1));
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SensorDataQoS(), std::bind(&HeadingStabilizer::imu_callback, this, std::placeholders::_1));

    pub_cmd_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_out", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&HeadingStabilizer::control_loop, this));

    RCLCPP_INFO(get_logger(), "Heading Stabilizer Online (Clean & Robust)");
  }

 private:
  void declare_parameters() {
    this->declare_parameter("enable_lock", true);
    this->declare_parameter("gyro_alpha", 0.3);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    double alpha = this->get_parameter("gyro_alpha").as_double();
    current_raw_rate_ = (alpha * msg->angular_velocity.z) + ((1.0 - alpha) * current_raw_rate_);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    current_yaw_ = 2.0 * atan2(qz, qw);
  }

  void cmd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    last_cmd_ = *msg;
    last_cmd_time_ = this->get_clock()->now();
    if (std::abs(msg->twist.angular.z) < 0.001 && !lock_active_) {
        target_yaw_lock_ = current_yaw_;
        lock_active_ = true;
    } else if (std::abs(msg->twist.angular.z) >= 0.001) {
        lock_active_ = false;
    }
  }

  void control_loop() {
    auto now = this->get_clock()->now();
    const double dt_s = 0.01;
    if ((now - last_cmd_time_).seconds() > 0.5) {
        pub_cmd_->publish(geometry_msgs::msg::TwistStamped());
        return;
    }

    auto out_msg = last_cmd_;
    out_msg.header.stamp = now;
    double target_rate = last_cmd_.twist.angular.z;

    if (lock_active_ && this->get_parameter("enable_lock").as_bool()) {
        double yaw_error = target_yaw_lock_ - current_yaw_;
        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
        target_rate = pid_heading_.compute_command(yaw_error, dt_s);
    }

    double rate_error = target_rate - current_raw_rate_;
    double final_correction = pid_rate_.compute_command(rate_error, dt_s);

    out_msg.twist.angular.z = target_rate + final_correction;
    pub_cmd_->publish(out_msg);
  }

  control_toolbox::Pid pid_heading_;
  control_toolbox::Pid pid_rate_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::TwistStamped last_cmd_;
  rclcpp::Time last_cmd_time_;
  double current_yaw_ = 0.0;
  double current_raw_rate_ = 0.0;
  double target_yaw_lock_ = 0.0;
  bool lock_active_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadingStabilizer>());
  rclcpp::shutdown();
  return 0;
}
