// Copyright 2026 Tatsukiyano
#include "mad_motor_driver/mad_motor_command_node.hpp"

#include <algorithm>
#include <cstdint>
#include "rclcpp_components/register_node_macro.hpp"

namespace
{
constexpr uint8_t kCanDataSize = 8;
}  // namespace

namespace mad_motor_driver
{

MadMotorCommandNode::MadMotorCommandNode(const rclcpp::NodeOptions & options)
: Node("mad_motor_command_node", options),
  latest_pwm_(0),
  last_pwm_time_(this->now())
{
  DeclareParameters();
  GetParameters();
  SetupRosInterfaces();

  RCLCPP_INFO(
    this->get_logger(),
    "MadMotorCommandNode started. pwm_topic=%s, can_tx_topic=%s, can_id=0x%X",
    pwm_topic_.c_str(),
    can_tx_topic_.c_str(),
    can_id_);
}

void MadMotorCommandNode::DeclareParameters()
{
  this->declare_parameter<std::string>("pwm_topic", "/shooter/cmd_muxed");
  this->declare_parameter<std::string>("can_tx_topic", "/can_tx");
  this->declare_parameter<int>("can_id", 0x201);
  this->declare_parameter<bool>("is_extended", false);
  this->declare_parameter<int>("min_pwm", -255);
  this->declare_parameter<int>("max_pwm", 255);
  this->declare_parameter<int>("send_period_ms", 20);
  this->declare_parameter<int>("timeout_ms", 500);
}

void MadMotorCommandNode::GetParameters()
{
  pwm_topic_ = this->get_parameter("pwm_topic").as_string();
  can_tx_topic_ = this->get_parameter("can_tx_topic").as_string();
  can_id_ = static_cast<uint32_t>(this->get_parameter("can_id").as_int());
  is_extended_ = this->get_parameter("is_extended").as_bool();
  min_pwm_ = static_cast<int>(this->get_parameter("min_pwm").as_int());
  max_pwm_ = static_cast<int>(this->get_parameter("max_pwm").as_int());
  send_period_ms_ = std::max(1, static_cast<int>(this->get_parameter("send_period_ms").as_int()));
  timeout_ms_ = std::max(1, static_cast<int>(this->get_parameter("timeout_ms").as_int()));

  if (min_pwm_ > max_pwm_) {
    RCLCPP_WARN(this->get_logger(), "min_pwm is greater than max_pwm. Swapping values.");
    std::swap(min_pwm_, max_pwm_);
  }
}

void MadMotorCommandNode::SetupRosInterfaces()
{
  pwm_subscription_ = this->create_subscription<std_msgs::msg::Int16>(
    pwm_topic_, 10,
    std::bind(&MadMotorCommandNode::PwmCallback, this, std::placeholders::_1));

  can_publisher_ = this->create_publisher<can_msgs::msg::Frame>(can_tx_topic_, 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(send_period_ms_),
    std::bind(&MadMotorCommandNode::TimerCallback, this));
}

void MadMotorCommandNode::PwmCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  latest_pwm_ = ClampPwm(msg->data);
  last_pwm_time_ = this->now();
}

void MadMotorCommandNode::TimerCallback()
{
  const rclcpp::Time now = this->now();
  int16_t pwm_to_send = GetPwmOrZeroOnTimeout(now);
  can_publisher_->publish(CreateCanFrame(pwm_to_send, now));
}

int16_t MadMotorCommandNode::ClampPwm(int value) const
{
  return static_cast<int16_t>(std::clamp(value, min_pwm_, max_pwm_));
}

int16_t MadMotorCommandNode::GetPwmOrZeroOnTimeout(const rclcpp::Time & now) const
{
  const auto elapsed = now - last_pwm_time_;
  if (elapsed > rclcpp::Duration::from_seconds(static_cast<double>(timeout_ms_) / 1000.0)) {
    return 0; // Timeout safety
  }
  return latest_pwm_;
}

can_msgs::msg::Frame MadMotorCommandNode::CreateCanFrame(
  int16_t pwm,
  const rclcpp::Time & stamp) const
{
  can_msgs::msg::Frame frame;
  frame.header.stamp = stamp;
  frame.id = can_id_;
  frame.is_rtr = false;
  frame.is_extended = is_extended_;
  frame.is_error = false;
  frame.dlc = kCanDataSize;

  frame.data.fill(0);
  frame.data[0] = static_cast<uint8_t>(pwm & 0xFF);
  frame.data[1] = static_cast<uint8_t>((pwm >> 8) & 0xFF);

  return frame;
}

}  // namespace mad_motor_driver

RCLCPP_COMPONENTS_REGISTER_NODE(mad_motor_driver::MadMotorCommandNode)
