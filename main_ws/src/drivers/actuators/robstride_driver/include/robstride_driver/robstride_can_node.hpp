// Copyright 2026 Tatsukiyano
#ifndef ROBSTRIDE_DRIVER__ROBSTRIDE_CAN_NODE_HPP_
#define ROBSTRIDE_DRIVER__ROBSTRIDE_CAN_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace robstride_driver
{

class RobstrideCanNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit RobstrideCanNode(const rclcpp::NodeOptions & options);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr message);
  void can_rx_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr message);

  uint8_t motor_id_;
  std::string joint_name_;
  bool invert_direction_;
  std::string topic_tx_queue_;
  std::string topic_rx_queue_;
  std::string topic_velocity_command_;

  double pos_min_ = -12.57, pos_max_ = 12.57;
  double vel_min_ = -50.0, vel_max_ = 50.0;
  double tor_min_ = -6.0, tor_max_ = 6.0;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_velocity_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_can_rx_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8MultiArray>::SharedPtr
    publisher_can_frames_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
    publisher_joint_state_;
};

}  // namespace robstride_driver

#endif  // ROBSTRIDE_DRIVER__ROBSTRIDE_CAN_NODE_HPP_
