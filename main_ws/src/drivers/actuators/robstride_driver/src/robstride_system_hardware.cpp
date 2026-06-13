// Copyright 2026 Tatsukiyano
#include "robstride_driver/robstride_system_hardware.hpp"

#include <cmath>
#include <algorithm>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robstride_driver/at_protocol_handler.hpp"
#include "robstride_driver/can_protocol_handler.hpp"
#include "robstride_driver/ddsm_protocol_handler.hpp"

namespace robstride_driver
{

hardware_interface::CallbackReturn RobstrideSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  node_ = std::make_shared<rclcpp::Node>("robstride_hardware_interface_node");

  // Determine Protocol
  std::string protocol_type = "at";
  if (info_.hardware_parameters.count("protocol")) {
    protocol_type = info_.hardware_parameters.at("protocol");
  }

  if (protocol_type == "at") {
    double max_speed_percentage = 50.0;
    if (info_.hardware_parameters.count("max_speed_limit_percentage")) {
      max_speed_percentage = std::stod(info_.hardware_parameters.at("max_speed_limit_percentage"));
    }
    int max_delta = static_cast<int>(at_protocol::NEUTRAL_VELOCITY_VALUE * (max_speed_percentage / 100.0));
    protocol_handler_ = std::make_unique<AtProtocolHandler>(vel_max_, max_delta);
  } else if (protocol_type == "can" || protocol_type == "ddsm") {
    // Both use CAN frames over serial
    if (protocol_type == "can") protocol_handler_ = std::make_unique<CanProtocolHandler>();
    else protocol_handler_ = std::make_unique<DdsmProtocolHandler>();
  } else {
    RCLCPP_FATAL(node_->get_logger(), "Unknown protocol type: %s", protocol_type.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize Transport
  transport_ = std::make_unique<seeed_usb_can::UsbCanSerialDriver>();
  transport_->set_receive_callback(std::bind(&RobstrideSystemHardware::can_rx_callback, this, std::placeholders::_1));

  // Initialize motors from URDF joints
  motors_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    motors_[i].id = std::stoi(joint.parameters.at("motor_id"), nullptr, 0);
    motors_[i].invert = (joint.parameters.at("invert_direction") == "true");
    id_to_index_[motors_[i].id] = i;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  seeed_usb_can::SerialDriverConfig config;
  if (info_.hardware_parameters.count("usb_path")) config.usb_path = info_.hardware_parameters.at("usb_path");
  if (info_.hardware_parameters.count("serial_baud")) config.serial_baud = std::stoi(info_.hardware_parameters.at("serial_baud"));
  
  try {
    transport_->open(config);
    RCLCPP_INFO(node_->get_logger(), "Transport opened on %s", config.usb_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node_->get_logger(), "Failed to open transport: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  transport_->close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (const auto & motor : motors_) {
    auto frame_data = protocol_handler_->create_enable_command(motor.id);
    if (!frame_data.empty()) {
      seeed_usb_can::CanFrame frame;
      frame.id = motor.id; // Simplified, assuming handler handles ID mapping if needed
      frame.data = frame_data;
      frame.dlc = static_cast<uint8_t>(frame_data.size());
      transport_->send_frame(frame);
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (const auto & motor : motors_) {
    auto frame_data = protocol_handler_->create_disable_command(motor.id);
    if (!frame_data.empty()) {
      seeed_usb_can::CanFrame frame;
      frame.id = motor.id;
      frame.data = frame_data;
      frame.dlc = static_cast<uint8_t>(frame_data.size());
      transport_->send_frame(frame);
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobstrideSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motors_[i].position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors_[i].velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &motors_[i].effort));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobstrideSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors_[i].command_velocity));
  }
  return command_interfaces;
}

hardware_interface::return_type RobstrideSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Non-blocking read is handled by can_rx_callback and mutex
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobstrideSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & motor : motors_) {
    double velocity = motor.command_velocity;
    if (motor.invert) velocity = -velocity;

    auto frame_data = protocol_handler_->create_velocity_command(motor.id, velocity);
    if (!frame_data.empty()) {
      seeed_usb_can::CanFrame frame;
      // Note: In real CAN, the ID might be different from motor.id depending on protocol
      // Here we assume motor.id is the CAN ID for simplicity.
      frame.id = motor.id;
      frame.data = frame_data;
      frame.dlc = static_cast<uint8_t>(frame_data.size());
      transport_->send_frame(frame);
    }
  }
  return hardware_interface::return_type::OK;
}

void RobstrideSystemHardware::can_rx_callback(const seeed_usb_can::CanFrame & frame) {
  auto result = protocol_handler_->decode_frame(frame.data);
  if (!result.success) return;

  uint8_t motor_id = result.motor_id;
  if (id_to_index_.find(motor_id) == id_to_index_.end()) return;

  size_t idx = id_to_index_[motor_id];
  auto & state = result.state;

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (motors_[idx].invert) {
    motors_[idx].position = -state.position;
    motors_[idx].velocity = -state.velocity;
    motors_[idx].effort = -state.effort;
  } else {
    motors_[idx].position = state.position;
    motors_[idx].velocity = state.velocity;
    motors_[idx].effort = state.effort;
  }
}

}  // namespace robstride_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robstride_driver::RobstrideSystemHardware, hardware_interface::SystemInterface)
