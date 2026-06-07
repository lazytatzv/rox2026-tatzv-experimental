// Copyright 2026 Tatsukiyano
#include "base_teleop/base_teleop_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace base_teleop {

BaseTeleopNode::BaseTeleopNode(const rclcpp::NodeOptions& options)
    : Node("base_teleop_node", options) {
  declare_parameters();
  cache_parameters();

  auto telemetry_qos = rclcpp::SystemDefaultsQoS();
  auto sensor_qos = rclcpp::SensorDataQoS();

  publisher_command_velocity_ =
      this->create_publisher<geometry_msgs::msg::Twist>(topic_cmd_vel_, telemetry_qos);

  publisher_stop_lock_ = this->create_publisher<std_msgs::msg::Bool>(topic_stop_lock_, telemetry_qos);

  subscription_joystick_ = this->create_subscription<sensor_msgs::msg::Joy>(
      topic_joy_, sensor_qos,
      std::bind(&BaseTeleopNode::joystick_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(20ms, std::bind(&BaseTeleopNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "BaseTeleopNode (3-Axis Pro) initialized.");
}

void BaseTeleopNode::declare_parameters() {
  // Axis Mappings (DualSense Standard)
  this->declare_parameter("joy_axis_linear_x", 1);    // Left Stick Vertical
  this->declare_parameter("joy_axis_linear_y", 0);    // Left Stick Horizontal (Strafe!)
  this->declare_parameter("joy_axis_angular_z", 2);   // Right Stick Horizontal (Rotate)
  
  this->declare_parameter("joy_button_software_stop", 15);
  this->declare_parameter("joy_button_joy_mode_on", 8);
  this->declare_parameter("scale_linear_velocity", 1.0);
  this->declare_parameter("scale_angular_velocity", 1.0);
  this->declare_parameter("smoothing_factor", 0.3);
  this->declare_parameter("topic_joy", "joy");
  this->declare_parameter("topic_cmd_vel", "cmd_vel_joy");
  this->declare_parameter("topic_stop_lock", "stop_lock");
}

void BaseTeleopNode::cache_parameters() {
  axis_linear_x_ = this->get_parameter("joy_axis_linear_x").as_int();
  axis_linear_y_ = this->get_parameter("joy_axis_linear_y").as_int();
  axis_angular_z_ = this->get_parameter("joy_axis_angular_z").as_int();
  
  button_software_stop_ = this->get_parameter("joy_button_software_stop").as_int();
  button_joy_mode_on_ = this->get_parameter("joy_button_joy_mode_on").as_int();
  scale_linear_velocity_ = this->get_parameter("scale_linear_velocity").as_double();
  scale_angular_velocity_ = this->get_parameter("scale_angular_velocity").as_double();
  smoothing_factor_ = std::clamp(this->get_parameter("smoothing_factor").as_double(), 0.01, 1.0);
  topic_joy_ = this->get_parameter("topic_joy").as_string();
  topic_cmd_vel_ = this->get_parameter("topic_cmd_vel").as_string();
  topic_stop_lock_ = this->get_parameter("topic_stop_lock").as_string();
}

void BaseTeleopNode::timer_callback() {
  if (!joy_mode_active_) return;

  auto smooth = [this](double current, double target) {
    return current + smoothing_factor_ * (target - current);
  };

  current_twist_.linear.x = smooth(current_twist_.linear.x, target_twist_.linear.x);
  current_twist_.linear.y = smooth(current_twist_.linear.y, target_twist_.linear.y);
  current_twist_.angular.z = smooth(current_twist_.angular.z, target_twist_.angular.z);

  auto apply_deadband = [](double val) { return (std::abs(val) < 0.001) ? 0.0 : val; };
  current_twist_.linear.x = apply_deadband(current_twist_.linear.x);
  current_twist_.linear.y = apply_deadband(current_twist_.linear.y);
  current_twist_.angular.z = apply_deadband(current_twist_.angular.z);

  auto msg = std::make_unique<geometry_msgs::msg::Twist>(current_twist_);
  publisher_command_velocity_->publish(std::move(msg));
}

void BaseTeleopNode::joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  size_t req_buttons = static_cast<size_t>(std::max(button_software_stop_, button_joy_mode_on_));
  if (msg->buttons.size() <= req_buttons) return;

  // STOP (Touchpad)
  if (msg->buttons[button_software_stop_] == 1) {
    joy_mode_active_ = false;
    RCLCPP_WARN(get_logger(), "SYSTEM DISARMED");
    return;
  }

  // ARM (Select)
  if (msg->buttons[button_joy_mode_on_] == 1) {
    joy_mode_active_ = true;
    RCLCPP_INFO(get_logger(), "SYSTEM ARMED (3-Axis Mode)");
  }

  if (!joy_mode_active_) return;

  size_t req_axes = static_cast<size_t>(std::max({axis_linear_x_, axis_linear_y_, axis_angular_z_}));
  if (msg->axes.size() <= req_axes) return;

  target_twist_.linear.x = msg->axes[axis_linear_x_] * scale_linear_velocity_;
  target_twist_.linear.y = msg->axes[axis_linear_y_] * scale_linear_velocity_;
  target_twist_.angular.z = msg->axes[axis_angular_z_] * scale_angular_velocity_;
}

}  // namespace base_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(base_teleop::BaseTeleopNode)
