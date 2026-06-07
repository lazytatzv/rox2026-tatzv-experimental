// Copyright 2026 Tatsukiyano
#include "robstride_driver/robstride_can_node.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robstride_driver {

RobstrideCanNode::RobstrideCanNode(const rclcpp::NodeOptions& options) 
: rclcpp_lifecycle::LifecycleNode("robstride_can_node", options) {
  this->declare_parameter("motor_id", 0x01);
  this->declare_parameter("joint_name", "motor_joint");
  this->declare_parameter("invert_direction", false);
  this->declare_parameter("topic_tx_queue", "/communication/tx_queue");
  this->declare_parameter("topic_rx_queue", "/communication/rx_queue");
  this->declare_parameter("topic_velocity_command", "~/velocity_command");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideCanNode::on_configure(const rclcpp_lifecycle::State &)
{
  motor_id_ = static_cast<uint8_t>(this->get_parameter("motor_id").as_int());
  joint_name_ = this->get_parameter("joint_name").as_string();
  invert_direction_ = this->get_parameter("invert_direction").as_bool();
  topic_tx_queue_ = this->get_parameter("topic_tx_queue").as_string();
  topic_rx_queue_ = this->get_parameter("topic_rx_queue").as_string();
  topic_velocity_command_ = this->get_parameter("topic_velocity_command").as_string();

  // --- QoS SYNCHRONIZATION ---
  auto telemetry_qos = rclcpp::SystemDefaultsQoS(); 
  auto sensor_qos = rclcpp::SensorDataQoS();
  auto command_qos = rclcpp::QoS(1).best_effort();

  publisher_can_frames_ = this->create_publisher<seeed_usb_can_analyzer_driver::msg::CanFrame>(topic_tx_queue_, sensor_qos);
  publisher_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", telemetry_qos);
  
  subscription_velocity_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    topic_velocity_command_, command_qos, std::bind(&RobstrideCanNode::velocity_callback, this, std::placeholders::_1));

  subscription_can_rx_ = this->create_subscription<seeed_usb_can_analyzer_driver::msg::CanFrame>(
    topic_rx_queue_, sensor_qos, std::bind(&RobstrideCanNode::can_rx_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Configured motor 0x%02X (CAN)", motor_id_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideCanNode::on_activate(const rclcpp_lifecycle::State &)
{
  publisher_can_frames_->on_activate();
  publisher_joint_state_->on_activate();
  RCLCPP_INFO(get_logger(), "Activated (CAN Mode)");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideCanNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  publisher_can_frames_->on_deactivate();
  publisher_joint_state_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated (CAN Mode)");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideCanNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  publisher_can_frames_.reset();
  publisher_joint_state_.reset();
  subscription_velocity_.reset();
  subscription_can_rx_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideCanNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void RobstrideCanNode::velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr message) {
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
  if (message->data.empty()) return;

  double velocity_rad_s = message->data[0];
  if (invert_direction_) velocity_rad_s = -velocity_rad_s;

  auto msg = std::make_unique<seeed_usb_can_analyzer_driver::msg::CanFrame>();
  msg->id = 0x400 + motor_id_;
  msg->extended = false;
  msg->remote = false;
  msg->dlc = 8;
  
  int32_t raw_vel = static_cast<int32_t>(velocity_rad_s * 1000.0);
  std::memcpy(&msg->data[0], &raw_vel, 4);

  publisher_can_frames_->publish(std::move(msg));
}

void RobstrideCanNode::can_rx_callback(const seeed_usb_can_analyzer_driver::msg::CanFrame::SharedPtr message) {
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
  if (message->id != static_cast<uint32_t>(0x500 + motor_id_)) return;

  int32_t pos_raw;
  int16_t vel_raw;
  int16_t tor_raw;
  std::memcpy(&pos_raw, &message->data[0], 4);
  std::memcpy(&vel_raw, &message->data[4], 2);
  std::memcpy(&tor_raw, &message->data[6], 2);

  double position = static_cast<double>(pos_raw) / 1000.0;
  double velocity = static_cast<double>(vel_raw) / 1000.0;
  double torque = static_cast<double>(tor_raw) / 1000.0;

  if (invert_direction_) {
    position = -position;
    velocity = -velocity;
    torque = -torque;
  }

  auto joint_state = std::make_unique<sensor_msgs::msg::JointState>();
  joint_state->header.stamp = this->now();
  joint_state->name.push_back(joint_name_);
  joint_state->position.push_back(position);
  joint_state->velocity.push_back(velocity);
  joint_state->effort.push_back(torque);

  publisher_joint_state_->publish(std::move(joint_state));
}

}  // namespace robstride_driver

RCLCPP_COMPONENTS_REGISTER_NODE(robstride_driver::RobstrideCanNode)
