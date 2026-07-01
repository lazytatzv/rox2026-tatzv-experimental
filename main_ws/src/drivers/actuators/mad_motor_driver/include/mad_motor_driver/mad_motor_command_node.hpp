// Copyright 2026 Tatsukiyano
#ifndef MAD_MOTOR_DRIVER__MAD_MOTOR_COMMAND_NODE_HPP_
#define MAD_MOTOR_DRIVER__MAD_MOTOR_COMMAND_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "can_msgs/msg/frame.hpp"

namespace mad_motor_driver
{

class MadMotorCommandNode : public rclcpp::Node
{
public:
  explicit MadMotorCommandNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MadMotorCommandNode() override = default;

private:
  void DeclareParameters();
  void GetParameters();
  void SetupRosInterfaces();
  void PwmCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void TimerCallback();
  int16_t ClampPwm(int value) const;
  int16_t GetPwmOrZeroOnTimeout(const rclcpp::Time & now) const;
  can_msgs::msg::Frame CreateCanFrame(int16_t pwm, const rclcpp::Time & stamp) const;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr pwm_subscription_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string pwm_topic_;
  std::string can_tx_topic_;
  uint32_t can_id_;
  bool is_extended_;
  int min_pwm_;
  int max_pwm_;
  int send_period_ms_;
  int timeout_ms_;

  int16_t latest_pwm_;
  rclcpp::Time last_pwm_time_;
};

}  // namespace mad_motor_driver

#endif  // MAD_MOTOR_DRIVER__MAD_MOTOR_COMMAND_NODE_HPP_
