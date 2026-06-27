// Copyright 2026 Tatsukiyano
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "robstride_driver/at_protocol_handler.hpp"
#include "robstride_driver/private_protocol_handler.hpp"
#include "seeed_usb_can_analyzer_driver/serial_protocol.hpp"

/**
 * @brief Professional CLI Tool for RobStride Motor ID configuration.
 * usage: ros2 run robstride_driver set_motor_id --old 127 --new 1 --port /dev/ttyUSB0 --protocol at
 */

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("set_motor_id_tool");

  // Declare parameters
  node->declare_parameter("old", 127);
  node->declare_parameter("new", 1);
  node->declare_parameter("port", "/dev/ttyUSB0");
  node->declare_parameter("protocol", "at");
  node->declare_parameter("baud", 115200);

  int old_id = node->get_parameter("old").as_int();
  int new_id = node->get_parameter("new").as_int();
  std::string port = node->get_parameter("port").as_string();
  std::string protocol = node->get_parameter("protocol").as_string();
  int baud = node->get_parameter("baud").as_int();

  std::cout << ">>> RobStride ID Configuration Tool <<<" << std::endl;
  std::cout << "Target Port: " << port << " (" << baud << " baud)" << std::endl;
  std::cout << "Protocol: " << protocol << std::endl;
  std::cout << "Action: Change Motor ID " << old_id << " -> " << new_id << std::endl;
  std::cout << "---------------------------------------" << std::endl;

  // 1. Initialize Protocol Handler
  std::unique_ptr<robstride_driver::RobstrideProtocol> handler;
  if (protocol == "at") {
    handler = std::make_unique<robstride_driver::AtProtocolHandler>(50.0, 16384);
  } else if (protocol == "can" || protocol == "private_can") {
    handler = std::make_unique<robstride_driver::PrivateProtocolHandler>();
  } else {
    std::cerr << "Error: Unknown protocol '" << protocol << "'. Use 'at', 'can', or 'private_can'." << std::endl;
    return 1;
  }

  // 2. Initialize Transport
  auto transport = std::make_unique<seeed_usb_can::UsbCanSerialDriver>();
  seeed_usb_can::SerialDriverConfig config;
  config.usb_path = port;
  config.serial_baud = baud;

  try {
    transport->open(config);
    std::cout << "Transport opened successfully." << std::endl;
  } catch (const std::exception & e) {
    std::cerr << "Error: Failed to open port: " << e.what() << std::endl;
    return 1;
  }

  // 3. Create and Send ID Set Command
  auto frame_data = handler->create_id_set_command(
    static_cast<uint8_t>(old_id),
    static_cast<uint8_t>(new_id)
  );

  if (frame_data.empty()) {
    std::cerr << "Error: ID change not supported by this protocol handler." << std::endl;
    return 1;
  }

  seeed_usb_can::CanFrame frame;
  frame.id = static_cast<uint32_t>(old_id);
  frame.data = frame_data;
  frame.dlc = static_cast<uint8_t>(frame_data.size());

  std::cout << "Sending ID change command..." << std::endl;
  transport->send_frame(frame);

  // Wait a bit for motor processing
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::cout << "---------------------------------------" << std::endl;
  std::cout << "Command sent. Please RESTART (Power Cycle) the motor" << std::endl;
  std::cout << "to apply the new ID permanently." << std::endl;
  std::cout << "Verify the change using: ros2 topic echo " << handler->get_default_rx_topic() <<
    std::endl;

  transport->close();
  rclcpp::shutdown();
  return 0;
}
