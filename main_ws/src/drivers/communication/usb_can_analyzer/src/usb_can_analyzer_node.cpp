// Copyright 2026 Tatsukiyano
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "custom_interfaces/msg/can_frame.hpp"
#include "seeed_usb_can_analyzer_driver/serial_protocol.hpp"

namespace seeed_usb_can
{

class UsbCanAnalyzerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit UsbCanAnalyzerNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("usb_can_analyzer", options)
  {
    declare_parameter<std::string>("usb_path", "/dev/ttyUSB0");
    declare_parameter<int>("serial_baud", 2000000);
    declare_parameter<int>("bitrate", 500000);
    declare_parameter<bool>("tx_extended", false);
    declare_parameter<int64_t>("filter_id", 0);
    declare_parameter<int64_t>("mask_id", 0);
    declare_parameter<int>("operation_mode", 0);
  }

  ~UsbCanAnalyzerNode() override
  {
    serial_driver_.close();
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    const std::string usb_path = get_parameter("usb_path").as_string();
    const int serial_baud = get_parameter("serial_baud").as_int();
    const int bitrate = get_parameter("bitrate").as_int();
    const bool tx_extended = get_parameter("tx_extended").as_bool();
    const int64_t filter_id = get_parameter("filter_id").as_int();
    const int64_t mask_id = get_parameter("mask_id").as_int();
    const int operation_mode = get_parameter("operation_mode").as_int();

    if (operation_mode < 0 || operation_mode > 3) {
      RCLCPP_ERROR(get_logger(), "Parameter 'operation_mode' must be 0..3");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    const auto can_baud_code = to_can_baud_code(bitrate);

    publisher_ = create_publisher<custom_interfaces::msg::CanFrame>(
      "/communication/rx", 100);

    subscription_ = create_subscription<custom_interfaces::msg::CanFrame>(
      "/communication/tx",
      100,
      [this](const custom_interfaces::msg::CanFrame::ConstSharedPtr msg) {
        this->handle_transmit(*msg);
      });

    config_.usb_path = usb_path;
    config_.serial_baud = serial_baud;
    config_.can_baud_code = can_baud_code;
    config_.tx_extended = tx_extended;
    config_.filter_id = static_cast<uint32_t>(filter_id);
    config_.mask_id = static_cast<uint32_t>(mask_id);
    config_.operation_mode = static_cast<uint8_t>(operation_mode);

    serial_driver_.set_receive_callback([this](const CanFrame & frame) {
        if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          auto msg = custom_interfaces::msg::CanFrame();
          msg.id = frame.id;
          msg.extended = frame.extended;
          msg.remote = frame.remote;
          msg.dlc = frame.dlc;
          msg.data = frame.data;
          publisher_->publish(msg);
        }
    });

    RCLCPP_INFO(get_logger(), "Configured on %s", usb_path.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    try {
      serial_driver_.open(config_);
      publisher_->on_activate();
      RCLCPP_INFO(get_logger(), "Activated and port opened");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    serial_driver_.close();
    publisher_->on_deactivate();
    RCLCPP_INFO(get_logger(), "Deactivated and port closed");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    publisher_.reset();
    subscription_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  static uint8_t to_can_baud_code(int bitrate)
  {
    switch (bitrate) {
      case 1000000: return 0x01;
      case 800000:  return 0x02;
      case 500000:  return 0x03;
      case 400000:  return 0x04;
      case 250000:  return 0x05;
      case 200000:  return 0x06;
      case 125000:  return 0x07;
      case 100000:  return 0x08;
      case 50000:   return 0x09;
      case 20000:   return 0x0A;
      case 10000:   return 0x0B;
      case 5000:    return 0x0C;
      default:      return 0x03; // Default 500k
    }
  }

  void handle_transmit(const custom_interfaces::msg::CanFrame & msg)
  {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    CanFrame frame;
    frame.id = msg.id;
    frame.extended = msg.extended;
    frame.remote = msg.remote;
    frame.dlc = msg.dlc;
    frame.data = msg.data;

    try {
      serial_driver_.send_frame(frame);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to transmit CAN frame: %s",
          e.what());
    }
  }

  UsbCanSerialDriver serial_driver_;
  SerialDriverConfig config_;
  rclcpp_lifecycle::LifecyclePublisher<custom_interfaces::msg::CanFrame>::SharedPtr publisher_;
  rclcpp::Subscription<custom_interfaces::msg::CanFrame>::SharedPtr subscription_;
};

}  // namespace seeed_usb_can

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(seeed_usb_can::UsbCanAnalyzerNode)
