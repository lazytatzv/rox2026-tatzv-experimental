// Copyright 2026 Tatsukiyano
#ifndef ROBSTRIDE_DRIVER__ROBSTRIDE_SYSTEM_HARDWARE_HPP_
#define ROBSTRIDE_DRIVER__ROBSTRIDE_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "robstride_driver/at_protocol.hpp"

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
  void serial_rx_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr message);
  void send_enable_command(uint8_t motor_id);
  void send_disable_command(uint8_t motor_id);
  double uint_to_float(uint16_t value, double low, double high);

  // ROS Node for serial communication
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_serial_tx_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_serial_rx_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  // State and Command values
  struct MotorState {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
    double command_velocity = 0.0;
    uint8_t id = 0;
    bool invert = false;
  };

  std::vector<MotorState> motors_;
  std::map<uint8_t, size_t> id_to_index_;

  // Physics constraints
  double pos_min_ = -12.57;
  double pos_max_ = 12.57;
  double vel_min_ = -50.0;
  double vel_max_ = 50.0;
  double tor_min_ = -6.0;
  double tor_max_ = 6.0;
  int max_at_command_delta_ = 16383; // 50% limit default
};

}  // namespace robstride_driver

#endif  // ROBSTRIDE_DRIVER__ROBSTRIDE_SYSTEM_HARDWARE_HPP_
  // ROBSTRIDE_DRIVER__ROBSTRIDE_SYSTEM_HARDWARE_HPP_
