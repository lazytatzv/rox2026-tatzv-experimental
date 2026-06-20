// Copyright 2026 Tatsukiyano
#ifndef BASE_TELEOP__BASE_TELEOP_NODE_HPP_
#define BASE_TELEOP__BASE_TELEOP_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

namespace base_teleop
{

class BaseTeleopNode : public rclcpp::Node {
public:
  explicit BaseTeleopNode(const rclcpp::NodeOptions & options);

private:
  void declare_parameters();
  void update_parameters();
  void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void timer_callback();

  // Subscriptions and Publishers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_joystick_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_command_velocity_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_stop_lock_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  int axis_linear_x_;
  int axis_linear_y_;
  int axis_angular_z_;
  int button_software_stop_;
  int button_joy_mode_on_;
  double scale_linear_x_;
  double scale_linear_y_;
  double scale_angular_velocity_;
  double smoothing_factor_;
  double joy_deadband_;
  std::string topic_joy_;
  std::string topic_cmd_vel_;
  std::string topic_stop_lock_;

  // Internal state
  geometry_msgs::msg::Twist current_twist_;
  geometry_msgs::msg::Twist target_twist_;
  bool joy_mode_active_ = false;
};

}  // namespace base_teleop

#endif  // BASE_TELEOP__BASE_TELEOP_NODE_HPP_
