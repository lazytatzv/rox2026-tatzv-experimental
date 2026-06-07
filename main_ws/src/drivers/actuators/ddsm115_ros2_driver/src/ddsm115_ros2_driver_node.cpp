// Copyright 2026 Tatsukiyano
#include "ddsm115_ros2_driver/ddsm115_ros2_driver_node.hpp"
#include <cmath>
#include <algorithm>
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace ddsm115_ros2_driver {

DDSM115DriverNode::DDSM115DriverNode(const rclcpp::NodeOptions& options) 
: rclcpp_lifecycle::LifecycleNode("ddsm115_driver_node", options) {
  this->declare_parameter("motor_id", 1);
  this->declare_parameter("joint_name", "motor_joint");
  this->declare_parameter("invert_direction", false);
  this->declare_parameter("topic_tx_queue", "/communication/tx_queue");
  this->declare_parameter("topic_rx_queue", "/communication/rx_queue");
  this->declare_parameter("topic_velocity_command", "~/velocity_command");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DDSM115DriverNode::on_configure(const rclcpp_lifecycle::State &)
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

  publisher_serial_frames_ = this->create_publisher<robot_interfaces::msg::SerialFrame>(topic_tx_queue_, sensor_qos);
  publisher_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", telemetry_qos);
  
  subscription_velocity_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    topic_velocity_command_, command_qos, std::bind(&DDSM115DriverNode::velocity_callback, this, std::placeholders::_1));

  subscription_serial_rx_ = this->create_subscription<robot_interfaces::msg::SerialFrame>(
    topic_rx_queue_, sensor_qos, std::bind(&DDSM115DriverNode::serial_rx_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Configured motor %d (DDSM)", motor_id_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DDSM115DriverNode::on_activate(const rclcpp_lifecycle::State &)
{
  publisher_serial_frames_->on_activate();
  publisher_joint_state_->on_activate();
  RCLCPP_INFO(get_logger(), "Activated (DDSM Mode)");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DDSM115DriverNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  publisher_serial_frames_->on_deactivate();
  publisher_joint_state_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated (DDSM Mode)");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DDSM115DriverNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  publisher_serial_frames_.reset();
  publisher_joint_state_.reset();
  subscription_velocity_.reset();
  subscription_serial_rx_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DDSM115DriverNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void DDSM115DriverNode::velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr message) {
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
  if (message->data.empty()) return;

  double velocity_rad_s = message->data[0];
  if (invert_direction_) velocity_rad_s = -velocity_rad_s;

  // DDSM specific RPM conversion
  double rpm = (velocity_rad_s * 60.0) / (2.0 * M_PI);
  
  auto frame = std::make_unique<robot_interfaces::msg::SerialFrame>();
  // Simple RPM command pack (Mock/Real logic needs to follow DDSM spec)
  frame->frame_data = {motor_id_, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  publisher_serial_frames_->publish(std::move(frame));
}

void DDSM115DriverNode::serial_rx_callback(const robot_interfaces::msg::SerialFrame::SharedPtr message) {
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
  const auto& data = message->frame_data;
  if (data.size() < 10 || data[0] != motor_id_) return;

  auto joint_state = std::make_unique<sensor_msgs::msg::JointState>();
  joint_state->header.stamp = this->now();
  joint_state->name.push_back(joint_name_);
  joint_state->position.push_back(0.0);
  joint_state->velocity.push_back(0.0);
  joint_state->effort.push_back(0.0);

  publisher_joint_state_->publish(std::move(joint_state));
}

}  // namespace ddsm115_ros2_driver

RCLCPP_COMPONENTS_REGISTER_NODE(ddsm115_ros2_driver::DDSM115DriverNode)
