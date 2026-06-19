// Copyright 2026 Tatsukiyano
#ifndef IMU_STABILIZER__STABILIZER_NODE_HPP_
#define IMU_STABILIZER__STABILIZER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "imu_stabilizer/heading_stabilizer_core.hpp"

namespace imu_stabilizer
{

class HeadingStabilizerNode : public rclcpp::Node
{
public:
  explicit HeadingStabilizerNode(const rclcpp::NodeOptions & options);

private:
  void declare_parameters();
  void update_config_from_params();

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & params);

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cmd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void control_loop();

  std::unique_ptr<HeadingStabilizerCore> core_;
  HeadingStabilizerConfig config_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  geometry_msgs::msg::TwistStamped last_cmd_;
  rclcpp::Time last_cmd_time_;
  double current_yaw_ = 0.0;
  double current_raw_rate_ = 0.0;
};

} // namespace imu_stabilizer

#endif // IMU_STABILIZER__STABILIZER_NODE_HPP_
