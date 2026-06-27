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
#include "can_msgs/msg/frame.hpp"
#include "robstride_driver/robstride_protocol.hpp"
#include "robstride_driver/robstride_serial_driver.hpp"

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
  void can_rx_callback(const std::vector<uint8_t> & frame);
  void can_rx_topic_callback(const can_msgs::msg::Frame::ConstSharedPtr msg);
  void send_command(uint8_t motor_id, const std::vector<uint8_t> & frame_data);
  void process_received_frame(const std::vector<uint8_t> & frame_data);

  // Configuration parameter
  std::string transport_type_{"serial"};
  std::string protocol_type_{"at"};

  // Protocol Handler
  std::unique_ptr<RobstrideProtocol> protocol_handler_;

  // Direct Communication Driver
  std::unique_ptr<RobstrideSerialDriver> transport_;

  // ROS Node for logging and parameters only
  rclcpp::Node::SharedPtr node_;

  // ROS Topic Communication
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  std::unique_ptr<std::thread> spin_thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // SocketCAN Direct Communication
  int can_socket_{-1};
  std::string can_interface_{"can0"};
  std::unique_ptr<std::thread> can_rx_thread_;
  std::atomic<bool> can_rx_running_{false};
  void can_rx_thread_func();

  // State and Command values
  struct Motor
  {
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
