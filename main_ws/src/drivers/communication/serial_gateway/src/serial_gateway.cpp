// Copyright 2026 Tatsukiyano
//
// Licensed under the MIT License;
// you may not use this file except in compliance with the License.

#include "serial_gateway/serial_gateway.hpp"

#include <boost/asio.hpp>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <iostream>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace serial_gateway {

SerialGateway::SerialGateway(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("serial_gateway", options), 
      is_connected_(false), tx_count_(0), rx_count_(0) {
  this->declare_parameter("serial_port", "/dev/ttyUSB1");
  this->declare_parameter("baud_rate", 921600);
  this->declare_parameter("reconnect_interval_ms", 2000);
}

SerialGateway::~SerialGateway() {
  if (io_context_) io_context_->stop();
  if (io_thread_.joinable()) io_thread_.join();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SerialGateway::on_configure(const rclcpp_lifecycle::State&) {
  diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostic_updater_->setHardwareID("Serial Bus");
  diagnostic_updater_->add("Connection Status", this, &SerialGateway::produce_diagnostics);

  auto sensor_qos = rclcpp::SensorDataQoS();
  publisher_rx_frames_ = this->create_publisher<robot_interfaces::msg::SerialFrame>("/communication/rx_queue", sensor_qos);
  subscription_serial_frames_ = this->create_subscription<robot_interfaces::msg::SerialFrame>(
      "/communication/tx_queue", sensor_qos, std::bind(&SerialGateway::serial_frame_callback, this, std::placeholders::_1));

  if (!io_context_) {
      io_context_ = std::make_unique<boost::asio::io_context>();
  }
  init_serial_port();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SerialGateway::on_activate(const rclcpp_lifecycle::State&) {
  publisher_rx_frames_->on_activate();

  int interval = this->get_parameter("reconnect_interval_ms").as_int();
  reconnect_timer_ = this->create_wall_timer(std::chrono::milliseconds(interval),
                                             std::bind(&SerialGateway::try_reconnect, this));

  if (!io_thread_.joinable()) {
    io_thread_ = std::thread([this]() {
      try {
        auto work = std::make_unique<boost::asio::io_context::work>(*io_context_);
        io_context_->run();
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "IO Context Error: %s", e.what());
      }
    });
  }

  start_async_read();
  RCLCPP_INFO(get_logger(), "Activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SerialGateway::on_deactivate(const rclcpp_lifecycle::State&) {
  reconnect_timer_.reset();
  publisher_rx_frames_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SerialGateway::on_cleanup(const rclcpp_lifecycle::State&) {
  if (io_context_) io_context_->stop();
  if (io_thread_.joinable()) io_thread_.join();
  if (serial_port_ && serial_port_->is_open()) serial_port_->close();
  
  serial_port_.reset();
  io_context_.reset(); // Force recreation on next configure
  
  subscription_serial_frames_.reset();
  publisher_rx_frames_.reset();
  diagnostic_updater_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SerialGateway::on_shutdown(const rclcpp_lifecycle::State&) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool SerialGateway::init_serial_port() {
  std::string port_path = this->get_parameter("serial_port").as_string();
  int baud_rate = this->get_parameter("baud_rate").as_int();

  try {
    if (serial_port_) {
        serial_port_->close();
        serial_port_.reset();
    }
    serial_port_ = std::make_unique<boost::asio::serial_port>(*io_context_, port_path);
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    
    is_connected_ = true;
    last_error_ = "None";
    RCLCPP_INFO(get_logger(), "Port %s opened successfully", port_path.c_str());
    return true;
  } catch (const std::exception& e) {
    is_connected_ = false;
    last_error_ = e.what();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Port %s offline (Retrying...)", port_path.c_str());
    return false;
  }
}

void SerialGateway::try_reconnect() {
  if (is_connected_) {
    if (!serial_port_ || !serial_port_->is_open()) is_connected_ = false;
    return;
  }
  if (init_serial_port()) start_async_read();
}

void SerialGateway::start_async_read() {
  if (!is_connected_ || !serial_port_ || !serial_port_->is_open()) return;

  boost::asio::async_read_until(
      *serial_port_, read_buffer_, "\r\n",
      [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
        if (!ec) {
          rx_count_++;
          if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            auto msg = std::make_unique<robot_interfaces::msg::SerialFrame>();
            std::istream is(&read_buffer_);
            msg->frame_data.resize(bytes_transferred);
            is.read(reinterpret_cast<char*>(msg->frame_data.data()), bytes_transferred);
            publisher_rx_frames_->publish(std::move(msg));
          }
          start_async_read();
        } else if (ec != boost::asio::error::operation_aborted) {
          is_connected_ = false;
          last_error_ = ec.message();
        }
      });
}

void SerialGateway::serial_frame_callback(const robot_interfaces::msg::SerialFrame::SharedPtr message) {
  if (!is_connected_ || !serial_port_ || !serial_port_->is_open()) return;
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;

  // Use async_write for thread-safe output from ROS 2 threads
  boost::asio::async_write(*serial_port_, boost::asio::buffer(message->frame_data),
      [this](const boost::system::error_code& ec, std::size_t /*length*/) {
          if (!ec) tx_count_++;
          else if (ec != boost::asio::error::operation_aborted) is_connected_ = false;
      });
}

void SerialGateway::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  if (is_connected_) stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Connected");
  else stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Hardware Disconnected");
  stat.add("Port", get_parameter("serial_port").as_string());
  stat.add("Baud Rate", get_parameter("baud_rate").as_int());
  stat.add("TX Frames", tx_count_);
  stat.add("RX Frames", rx_count_);
}

}  // namespace serial_gateway

RCLCPP_COMPONENTS_REGISTER_NODE(serial_gateway::SerialGateway)
