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
    RCLCPP_INFO(node_->get_logger(), "Using AT Protocol Handler");
  } else if (protocol_type == "can") {
    protocol_handler_ = std::make_unique<CanProtocolHandler>();
    RCLCPP_INFO(node_->get_logger(), "Using CAN Protocol Handler");
  } else {
    RCLCPP_FATAL(node_->get_logger(), "Unknown protocol type: %s", protocol_type.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Setup Topics
  std::string tx_topic = protocol_handler_->get_default_tx_topic();
  std::string rx_topic = protocol_handler_->get_default_rx_topic();
  if (info_.hardware_parameters.count("topic_tx_queue")) tx_topic = info_.hardware_parameters.at("topic_tx_queue");
  if (info_.hardware_parameters.count("topic_rx_queue")) rx_topic = info_.hardware_parameters.at("topic_rx_queue");

  auto sensor_qos = rclcpp::SensorDataQoS();
  publisher_tx_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(tx_topic, sensor_qos);
  subscription_rx_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
    rx_topic, sensor_qos,
    std::bind(&RobstrideSystemHardware::rx_callback, this, std::placeholders::_1));

  // Initialize motors from URDF joints
  motors_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(node_->get_logger(), "Joint '%s' requires exactly one command interface: 'velocity'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    motors_[i].id = std::stoi(joint.parameters.at("motor_id"), nullptr, 0);
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
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  for (const auto & motor : motors_) {
    auto frame = protocol_handler_->create_enable_command(motor.id);
    if (!frame.empty()) {
      auto msg = std::make_unique<std_msgs::msg::UInt8MultiArray>();
      msg->data = frame;
      publisher_tx_->publish(std::move(msg));
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "RobstrideSystemHardware activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (const auto & motor : motors_) {
    auto frame = protocol_handler_->create_disable_command(motor.id);
    if (!frame.empty()) {
      auto msg = std::make_unique<std_msgs::msg::UInt8MultiArray>();
      msg->data = frame;
      publisher_tx_->publish(std::move(msg));
    }
  }

  if (executor_) {
    executor_->cancel();
    if (executor_thread_.joinable()) executor_thread_.join();
  }

  RCLCPP_INFO(node_->get_logger(), "RobstrideSystemHardware deactivated!");
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
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobstrideSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & motor : motors_) {
    double velocity = motor.command_velocity;
    if (motor.invert) velocity = -velocity;

    auto frame = protocol_handler_->create_velocity_command(motor.id, velocity);
    if (!frame.empty()) {
      auto msg = std::make_unique<std_msgs::msg::UInt8MultiArray>();
      msg->data = frame;
      publisher_tx_->publish(std::move(msg));
    }
  }
  return hardware_interface::return_type::OK;
}

void RobstrideSystemHardware::rx_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr message) {
  auto result = protocol_handler_->decode_frame(message->data);
  if (!result) return;

  uint8_t motor_id = result->first;
  if (id_to_index_.find(motor_id) == id_to_index_.end()) return;

  size_t idx = id_to_index_[motor_id];
  auto & state = result->second;

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
