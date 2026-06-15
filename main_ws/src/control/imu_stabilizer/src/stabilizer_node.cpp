// Copyright 2026 Tatsukiyano
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "imu_stabilizer/heading_stabilizer_core.hpp"

using namespace std::chrono_literals;

namespace imu_stabilizer
{

class HeadingStabilizerNode : public rclcpp::Node
{
public:
  HeadingStabilizerNode() : Node("heading_stabilizer")
  {
    declare_parameters();
    
    HeadingStabilizerCore::Config config;
    config.gyro_alpha = this->get_parameter("gyro_alpha").as_double();
    // In a real pro setup, we would read PID gains from parameters here
    
    core_ = std::make_unique<HeadingStabilizerCore>(config);

    sub_cmd_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel_in", 10, std::bind(&HeadingStabilizerNode::cmd_callback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10, std::bind(&HeadingStabilizerNode::odom_callback, this, std::placeholders::_1));
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(), std::bind(&HeadingStabilizerNode::imu_callback, this, std::placeholders::_1));

    pub_cmd_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_out", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&HeadingStabilizerNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "Heading Stabilizer Online (Modular & Unit-Tested Architecture)");
  }

private:
  void declare_parameters()
  {
    this->declare_parameter("enable_lock", true);
    this->declare_parameter("gyro_alpha", 0.3);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    current_raw_rate_ = msg->angular_velocity.z;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    current_yaw_ = 2.0 * atan2(qz, qw);
  }

  void cmd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    last_cmd_ = *msg;
    last_cmd_time_ = this->get_clock()->now();
    core_->updateCommand(msg->twist.angular.z, current_yaw_);
  }

  void control_loop()
  {
    auto now = this->get_clock()->now();
    const double dt_s = 0.01;

    if ((now - last_cmd_time_).seconds() > 0.5) {
      pub_cmd_->publish(geometry_msgs::msg::TwistStamped());
      return;
    }

    if (!this->get_parameter("enable_lock").as_bool()) {
      pub_cmd_->publish(last_cmd_);
      return;
    }

    double out_rate = core_->compute(current_raw_rate_, current_yaw_, dt_s);

    auto out_msg = last_cmd_;
    out_msg.header.stamp = now;
    out_msg.twist.angular.z = (core_->isLockActive()) ? out_rate : last_cmd_.twist.angular.z;
    
    pub_cmd_->publish(out_msg);
  }

  std::unique_ptr<HeadingStabilizerCore> core_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::TwistStamped last_cmd_;
  rclcpp::Time last_cmd_time_;
  double current_yaw_ = 0.0;
  double current_raw_rate_ = 0.0;
};

} // namespace imu_stabilizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_stabilizer::HeadingStabilizerNode>());
  rclcpp::shutdown();
  return 0;
}
