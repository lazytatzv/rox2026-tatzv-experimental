// Copyright 2026 Tatsukiyano
#ifndef MECANUM_KINEMATICS__MECANUM_KINEMATICS_NODE_HPP_
#define MECANUM_KINEMATICS__MECANUM_KINEMATICS_NODE_HPP_

#include <memory>
#include <vector>
#include <string>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace mecanum_kinematics
{

class MecanumKinematicsNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit MecanumKinematicsNode(const rclcpp::NodeOptions & options);

  // Lifecycle Transitions
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
  void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void declare_parameters();
  void update_parameters();
  void watchdog_callback();
  void publish_wheel_commands(const std::array<double, 4>& speeds);

  // Robot Geometry
  double half_length_;
  double half_width_;
  double wheel_radius_;

  // Safety & State
  double x_ = 0.0;
  double y_ = 0.0;
  double th_ = 0.0;
  rclcpp::Time last_time_;
  rclcpp::Time last_command_time_;
  bool first_odom_ = true;
  double watchdog_timeout_ = 1.0;
  bool watchdog_triggered_ = false;

  std::string topic_cmd_vel_;

  // Lifecycle Publishers
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_;
  
  // Direct Motor Publishers (Consolidated from Dispatcher)
  std::array<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr, 4> motor_pubs_;

  // Subscriptions & Timers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_command_velocity_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_states_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace mecanum_kinematics

#endif  // MECANUM_KINEMATICS__MECANUM_KINEMATICS_NODE_HPP_
