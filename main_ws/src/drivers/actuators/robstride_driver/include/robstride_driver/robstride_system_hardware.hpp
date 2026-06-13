// Copyright 2026 Tatsukiyano
#ifndef ROBSTRIDE_DRIVER__ROBSTRIDE_SYSTEM_HARDWARE_HPP_
#define ROBSTRIDE_DRIVER__ROBSTRIDE_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <mutex>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "robstride_driver/robstride_protocol.hpp"
#include "seeed_usb_can_analyzer_driver/serial_protocol.hpp"

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
  void can_rx_callback(const seeed_usb_can::CanFrame & frame);

  // Protocol Handler
  std::unique_ptr<RobstrideProtocol> protocol_handler_;

  // Direct Communication Driver
  std::unique_ptr<seeed_usb_can::UsbCanSerialDriver> transport_;

  // ROS Node for logging and parameters only
  rclcpp::Node::SharedPtr node_;

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

  // Real-time safety
  std::mutex state_mutex_;
};

}  // namespace robstride_driver

#endif  // ROBSTRIDE_DRIVER__ROBSTRIDE_SYSTEM_HARDWARE_HPP_
