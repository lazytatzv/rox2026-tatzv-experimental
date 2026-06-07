// Copyright 2026 Tatsukiyano
#ifndef DDSM115_ROS2_DRIVER__DDSM115_ROS2_DRIVER_NODE_HPP_
#define DDSM115_ROS2_DRIVER__DDSM115_ROS2_DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace ddsm115_ros2_driver
{

class DDSM115DriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit DDSM115DriverNode(const rclcpp::NodeOptions & options);

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
  void serial_rx_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr message);

  uint8_t motor_id_;
  std::string joint_name_;
  bool invert_direction_;
  std::string topic_tx_queue_;
  std::string topic_rx_queue_;
  std::string topic_velocity_command_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_serial_frames_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_state_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_velocity_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_serial_rx_;
};

}  // namespace ddsm115_ros2_driver

#endif  // DDSM115_ROS2_DRIVER__DDSM115_ROS2_DRIVER_NODE_HPP_
