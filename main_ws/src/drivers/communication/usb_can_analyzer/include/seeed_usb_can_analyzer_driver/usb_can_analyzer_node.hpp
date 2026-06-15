// Copyright 2026 Tatsukiyano
#ifndef SEEED_USB_CAN_ANALYZER_DRIVER__USB_CAN_ANALYZER_NODE_HPP_
#define SEEED_USB_CAN_ANALYZER_DRIVER__USB_CAN_ANALYZER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "seeed_usb_can_analyzer_driver/msg/can_frame.hpp"
#include "seeed_usb_can_analyzer_driver/serial_protocol.hpp"

namespace seeed_usb_can
{

class UsbCanAnalyzerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit UsbCanAnalyzerNode(const rclcpp::NodeOptions & options);
  virtual ~UsbCanAnalyzerNode();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
  static uint8_t to_can_baud_code(int bitrate);
  void handle_transmit(const seeed_usb_can_analyzer_driver::msg::CanFrame & msg);

  UsbCanSerialDriver serial_driver_;
  SerialDriverConfig config_;
  rclcpp_lifecycle::LifecyclePublisher<seeed_usb_can_analyzer_driver::msg::CanFrame>::SharedPtr
    publisher_;
  rclcpp::Subscription<seeed_usb_can_analyzer_driver::msg::CanFrame>::SharedPtr subscription_;
};

}  // namespace seeed_usb_can

#endif  // SEEED_USB_CAN_ANALYZER_DRIVER__USB_CAN_ANALYZER_NODE_HPP_
