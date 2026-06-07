// Copyright 2026 Tatsukiyano
#include "robstride_driver/robstride_can_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robstride_driver {

RobstrideCanNode::RobstrideCanNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("robstride_can_node", options) {
  this->declare_parameter("motor_id", 0x01);
  this->declare_parameter("joint_name", "motor_joint");
  this->declare_parameter("invert_direction", false);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideCanNode::on_configure(const rclcpp_lifecycle::State&) {
  motor_id_ = this->get_parameter("motor_id").as_int();
  joint_name_ = this->get_parameter("joint_name").as_string();
  invert_direction_ = this->get_parameter("invert_direction").as_bool();

  auto sensor_qos = rclcpp::SensorDataQoS();
  auto command_qos = rclcpp::QoS(1).best_effort();

  publisher_can_tx_ = this->create_publisher<seeed_usb_can_analyzer_driver::msg::CanFrame>(
      "/communication/tx_queue", command_qos);
  publisher_joint_state_ =
      this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", sensor_qos);

  subscription_velocity_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/velocity_command", command_qos,
      std::bind(&RobstrideCanNode::velocity_callback, this, std::placeholders::_1));

  subscription_can_rx_ = this->create_subscription<seeed_usb_can_analyzer_driver::msg::CanFrame>(
      "/communication/rx_queue", sensor_qos,
      std::bind(&RobstrideCanNode::can_rx_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Configured for Seeed RobStride Protocol (MotorID: %d)",
              static_cast<int>(motor_id_));
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideCanNode::on_activate(const rclcpp_lifecycle::State&) {
  publisher_can_tx_->on_activate();
  publisher_joint_state_->on_activate();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideCanNode::on_deactivate(const rclcpp_lifecycle::State&) {
  publisher_can_tx_->on_deactivate();
  publisher_joint_state_->on_deactivate();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideCanNode::on_cleanup(const rclcpp_lifecycle::State&) {
  publisher_can_tx_.reset();
  publisher_joint_state_.reset();
  subscription_velocity_.reset();
  subscription_can_rx_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void RobstrideCanNode::velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr message) {
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }
  if (message->data.empty()) {
    return;
  }

  double velocity = message->data[0];
  if (invert_direction_) {
    velocity = -velocity;
  }

  // --- SEEED WIKI SPEC: Speed Mode Control (0x400 + ID) ---
  seeed_usb_can_analyzer_driver::msg::CanFrame frame;
  frame.id = 0x400 + (motor_id_ & 0x7F);
  frame.extended = true;
  frame.dlc = 8;
  frame.data.resize(8, 0);

  // Velocity scaled by 1000 as per Wiki example
  int32_t v_int = static_cast<int32_t>(velocity * 1000.0);
  std::memcpy(&frame.data[0], &v_int, sizeof(int32_t)); // Little Endian

  // Optional: Max Torque (Placeholder if needed, let's keep it 0 as per basic usage)
  
  publisher_can_tx_->publish(frame);
}

void RobstrideCanNode::can_rx_callback(const seeed_usb_can_analyzer_driver::msg::CanFrame::SharedPtr message) {
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  // --- SEEED WIKI SPEC: Status Feedback (0x500 + ID) ---
  if (message->id != (0x500 + motor_id_)) {
    return;
  }
  if (message->data.size() < 8) {
    return;
  }

  // struct motor_status_t: pos(4), vel(2), tor(2)
  int32_t p_raw;
  int16_t v_raw, t_raw;
  std::memcpy(&p_raw, &message->data[0], 4);
  std::memcpy(&v_raw, &message->data[4], 2);
  std::memcpy(&t_raw, &message->data[6], 2);

  // Scaling based on Seeed Wiki
  double position = static_cast<double>(p_raw) / 1000.0;
  double velocity = static_cast<double>(v_raw) / 1000.0;
  double torque = static_cast<double>(t_raw) / 1000.0;

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
