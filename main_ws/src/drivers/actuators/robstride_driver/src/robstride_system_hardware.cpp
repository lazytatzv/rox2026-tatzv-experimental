// Copyright 2026 Tatsukiyano
#include "robstride_driver/robstride_system_hardware.hpp"

#include <cmath>
#include <algorithm>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robstride_driver
{

using namespace at_protocol;

hardware_interface::CallbackReturn RobstrideSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse parameters from URDF
  double max_speed_percentage = 50.0;
  if (info_.hardware_parameters.count("max_speed_limit_percentage")) {
    max_speed_percentage = std::stod(info_.hardware_parameters.at("max_speed_limit_percentage"));
  }
  max_at_command_delta_ = static_cast<int>(NEUTRAL_VELOCITY_VALUE * (max_speed_percentage / 100.0));

  std::string serial_write_topic = "/serial_write";
  std::string serial_read_topic = "/serial_read";
  if (info_.hardware_parameters.count("topic_tx_queue")) {
      serial_write_topic = info_.hardware_parameters.at("topic_tx_queue");
  }
  if (info_.hardware_parameters.count("topic_rx_queue")) {
      serial_read_topic = info_.hardware_parameters.at("topic_rx_queue");
  }

  // Setup ROS node for serial communication
  node_ = std::make_shared<rclcpp::Node>("robstride_hardware_interface_node");
  
  auto sensor_qos = rclcpp::SensorDataQoS();
  publisher_serial_tx_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(serial_write_topic, sensor_qos);
  subscription_serial_rx_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
    serial_read_topic, sensor_qos,
    std::bind(&RobstrideSystemHardware::serial_rx_callback, this, std::placeholders::_1));

  // Initialize motors from URDF joints
  motors_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    
    // Check interfaces
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(node_->get_logger(), "Joint '%s' requires exactly one command interface: 'velocity'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    motors_[i].id = std::stoi(joint.parameters.at("motor_id"), nullptr, 16);
    motors_[i].invert = (joint.parameters.at("invert_direction") == "true");
    
    id_to_index_[motors_[i].id] = i;
    RCLCPP_INFO(node_->get_logger(), "Initialized joint '%s' (ID: 0x%02X, Invert: %d)", joint.name.c_str(), motors_[i].id, motors_[i].invert);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Start the executor for serial callbacks
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  // Enable all motors
  for (const auto & motor : motors_) {
    send_enable_command(motor.id);
  }
  
  RCLCPP_INFO(node_->get_logger(), "RobstrideSystemHardware successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Disable all motors
  for (const auto & motor : motors_) {
    send_disable_command(motor.id);
  }

  // Stop executor
  if (executor_) {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }

  RCLCPP_INFO(node_->get_logger(), "RobstrideSystemHardware successfully deactivated!");
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
  // Callbacks update the state asynchronously, so nothing strict to do here.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobstrideSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & motor : motors_) {
    double velocity_rad_s = motor.command_velocity;
    if (motor.invert) velocity_rad_s = -velocity_rad_s;
    velocity_rad_s = std::clamp(velocity_rad_s, vel_min_, vel_max_);

    int delta = static_cast<int>(std::round((velocity_rad_s / vel_max_) * max_at_command_delta_));
    uint16_t at_value = NEUTRAL_VELOCITY_VALUE + delta;
    uint8_t direction_flag = (at_value == NEUTRAL_VELOCITY_VALUE) ? DIR_STOP : DIR_ROTATING;

    auto msg = std::make_unique<std_msgs::msg::UInt8MultiArray>();
    msg->data = {
      FRAME_HEADER_A, FRAME_HEADER_T, CMD_DATA_STREAMING,
      DEFAULT_SOURCE_ID_HI, DEFAULT_SOURCE_ID_LO, motor.id,
      DATA_LEN_8_BYTES, SPEED_CMD_INDICATOR, REG_ADDR_VELOCITY_CTRL,
      0x00, 0x00, CTRL_MODE_VELOCITY, direction_flag,
      static_cast<uint8_t>((at_value >> 8) & 0xFF),
      static_cast<uint8_t>(at_value & 0xFF),
      FRAME_FOOTER_CR, FRAME_FOOTER_LF
    };
    publisher_serial_tx_->publish(std::move(msg));
  }
  return hardware_interface::return_type::OK;
}

void RobstrideSystemHardware::send_enable_command(uint8_t motor_id) {
  auto msg = std::make_unique<std_msgs::msg::UInt8MultiArray>();
  msg->data = {
    FRAME_HEADER_A, FRAME_HEADER_T, CMD_BASIC_CONFIG,
    DEFAULT_SOURCE_ID_HI, DEFAULT_SOURCE_ID_LO, motor_id,
    DATA_LEN_8_BYTES, 0x00, REG_ADDR_MOTOR_ENABLE,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    FRAME_FOOTER_CR, FRAME_FOOTER_LF
  };
  publisher_serial_tx_->publish(std::move(msg));
}

void RobstrideSystemHardware::send_disable_command(uint8_t motor_id) {
  auto msg = std::make_unique<std_msgs::msg::UInt8MultiArray>();
  msg->data = {
    FRAME_HEADER_A, FRAME_HEADER_T, CMD_BASIC_CONFIG,
    DEFAULT_SOURCE_ID_HI, DEFAULT_SOURCE_ID_LO, motor_id,
    DATA_LEN_8_BYTES, 0x00, REG_ADDR_MOTOR_ENABLE,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    FRAME_FOOTER_CR, FRAME_FOOTER_LF
  };
  publisher_serial_tx_->publish(std::move(msg));
}

double RobstrideSystemHardware::uint_to_float(uint16_t value, double low, double high) {
  double span = high - low;
  return static_cast<double>(value) * span / 65535.0 + low;
}

void RobstrideSystemHardware::serial_rx_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr message) {
  const auto& data = message->data;
  
  if (data.size() < 16) return;
  if (data[0] != FRAME_HEADER_A || data[1] != FRAME_HEADER_T) return;
  
  uint8_t motor_id = data[5];
  if (id_to_index_.find(motor_id) == id_to_index_.end()) return;

  size_t idx = id_to_index_[motor_id];

  uint16_t pos_u = (data[7] << 8) | data[8];
  uint16_t vel_u = (data[9] << 8) | data[10];
  uint16_t tor_u = (data[11] << 8) | data[12];

  double position = uint_to_float(pos_u, pos_min_, pos_max_);
  double velocity = uint_to_float(vel_u, vel_min_, vel_max_);
  double torque = uint_to_float(tor_u, tor_min_, tor_max_);

  if (motors_[idx].invert) {
    position = -position;
    velocity = -velocity;
    torque = -torque;
  }

  motors_[idx].position = position;
  motors_[idx].velocity = velocity;
  motors_[idx].effort = torque;
}

}  // namespace robstride_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robstride_driver::RobstrideSystemHardware, hardware_interface::SystemInterface)
