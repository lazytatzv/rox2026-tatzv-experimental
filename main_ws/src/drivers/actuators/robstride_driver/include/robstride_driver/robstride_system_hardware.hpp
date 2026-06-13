// Copyright 2026 Tatsukiyano
#ifndef ROBSTRIDE_DRIVER__ROBSTRIDE_SYSTEM_HARDWARE_HPP_
#define ROBSTRIDE_DRIVER__ROBSTRIDE_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "robstride_driver/robstride_protocol.hpp"

namespace robstride_driver
{

class RobstrideSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobstrideSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void rx_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr message);

  // Protocol Handler
  std::unique_ptr<RobstrideProtocol> protocol_handler_;

  // ROS Node for communication
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_tx_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_rx_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  // State and Command values
  struct Motor {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
    double command_velocity = 0.0;
    uint8_t id = 0;
    bool invert = false;
  };

  std::vector<Motor> motors_;
  std::map<uint8_t, size_t> id_to_index_;

  // Physics constraints
  double vel_max_ = 50.0;
};

}  // namespace robstride_driver

#endif  // ROBSTRIDE_DRIVER__ROBSTRIDE_SYSTEM_HARDWARE_HPP_
