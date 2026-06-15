// Copyright 2026 Tatsukiyano
#include "virtual_actuator/virtual_system_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <random>

namespace virtual_actuator
{

hardware_interface::CallbackReturn VirtualSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  motors_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    motors_[i].inertia = 0.5;
    motors_[i].friction = 0.01;
    motors_[i].noise_stddev = 0.001;
    // LPF Alpha: 0.1 means 90% old data, 10% new data (Approx 10Hz cutoff at 100Hz)
    motors_[i].lpf_alpha = 0.15;

    if (info_.hardware_parameters.count("inertia")) {
      motors_[i].inertia = std::stod(info_.hardware_parameters.at("inertia"));
    }
    if (info_.hardware_parameters.count("lpf_alpha")) {
      motors_[i].lpf_alpha = std::stod(info_.hardware_parameters.at("lpf_alpha"));
    }
  }

  gen_ = std::mt19937(rd_());
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VirtualSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motors_[i].position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors_[i].filtered_velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &motors_[i].effort));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VirtualSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors_[i].command_velocity));
  }
  return command_interfaces;
}

hardware_interface::return_type VirtualSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  for (auto & motor : motors_) {
    double target = motor.command_velocity;
    double friction_loss = (motor.velocity > 0) ? motor.friction : (motor.velocity <
      0 ? -motor.friction : 0);

    // Physics simulation (Raw velocity)
    double dv = (target - motor.velocity) * motor.inertia - friction_loss;
    motor.velocity += dv * dt;
    motor.position += motor.velocity * dt;

    // Sensor Noise
    std::normal_distribution<double> dist(0, motor.noise_stddev);
    double raw_vel = motor.velocity + dist(gen_);

    // --- PROFESSIONAL VELOCITY FILTER ---
    // Low-pass filter (Exponential Moving Average)
    motor.filtered_velocity = (motor.lpf_alpha * raw_vel) +
      ((1.0 - motor.lpf_alpha) * motor.filtered_velocity);

    motor.effort = dv;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VirtualSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

}  // namespace virtual_actuator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  virtual_actuator::VirtualSystemHardware, hardware_interface::SystemInterface)
