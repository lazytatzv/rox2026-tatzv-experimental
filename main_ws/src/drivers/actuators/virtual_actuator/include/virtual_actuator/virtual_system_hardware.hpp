// Copyright 2026 Tatsukiyano
#ifndef VIRTUAL_ACTUATOR__VIRTUAL_SYSTEM_HARDWARE_HPP_
#define VIRTUAL_ACTUATOR__VIRTUAL_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

namespace virtual_actuator
{

class VirtualSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(VirtualSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct VirtualMotorState {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
    double command_velocity = 0.0;
    double inertia = 0.1;
  };

  std::vector<VirtualMotorState> motors_;
};

}  // namespace virtual_actuator

#endif  // VIRTUAL_ACTUATOR__VIRTUAL_SYSTEM_HARDWARE_HPP_
