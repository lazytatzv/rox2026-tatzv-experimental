// Copyright 2026 Tatsukiyano
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

namespace virtual_actuator {

class VirtualActuatorNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit VirtualActuatorNode(const rclcpp::NodeOptions& options)
      : rclcpp_lifecycle::LifecycleNode("virtual_actuator_node", options) {
    declare_parameter("joint_name", "virtual_joint");
    declare_parameter("inertia", 0.1);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State&) override {
    joint_name_ = get_parameter("joint_name").as_string();
    inertia_ = get_parameter("inertia").as_double();

    // --- QoS SYNCHRONIZATION ---
    auto telemetry_qos = rclcpp::SystemDefaultsQoS();
    auto command_qos = rclcpp::QoS(1).best_effort();

    publisher_joint_state_ =
        this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", telemetry_qos);

    subscription_velocity_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/velocity_command", command_qos,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          if (msg->data.size() > 0) target_velocity_ = msg->data[0];
        });

    timer_ = create_wall_timer(20ms, std::bind(&VirtualActuatorNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Configured Virtual Actuator: %s", joint_name_.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State&) override {
    publisher_joint_state_->on_activate();
    RCLCPP_INFO(get_logger(), "Activated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State&) override {
    publisher_joint_state_->on_deactivate();
    RCLCPP_INFO(get_logger(), "Deactivated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State&) override {
    publisher_joint_state_.reset();
    subscription_velocity_.reset();
    timer_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State&) override {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

 private:
  void timer_callback() {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    // Simple 1st order inertia
    current_velocity_ += (target_velocity_ - current_velocity_) * inertia_;
    current_position_ += current_velocity_ * 0.02;

    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->header.stamp = now();
    msg->name.push_back(joint_name_);
    msg->position.push_back(current_position_);
    msg->velocity.push_back(current_velocity_);
    msg->effort.push_back(0.0);

    publisher_joint_state_->publish(std::move(msg));
  }

  std::string joint_name_;
  double inertia_;
  double target_velocity_ = 0.0;
  double current_velocity_ = 0.0;
  double current_position_ = 0.0;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
      publisher_joint_state_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_velocity_;
};

}  // namespace virtual_actuator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(virtual_actuator::VirtualActuatorNode)
