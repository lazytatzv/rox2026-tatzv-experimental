// Copyright 2026 Tatsukiyano
#include "ddsm115_ros2_driver/ddsm115_ros2_driver_client.hpp"

#include <bit>
#include <cmath>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

constexpr uint32_t BAUD_RATE = 115200;

namespace ddsm115_ros2_driver {

static uint8_t calc_crc8_maxim(const std::vector<uint8_t> &data) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < data.size() - 1; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x01)
        crc = (crc >> 1) ^ 0x8C;
      else
        crc >>= 1;
    }
  }
  return crc;
}

DDSM115DriverClient::DDSM115DriverClient(FeedbackCallback feedback_callback,
                                         LogCallback log_callback)
    : baud_rate_(BAUD_RATE),
      feedback_callback_(std::move(feedback_callback)),
      log_callback_(std::move(log_callback)) {}

DDSM115DriverClient::~DDSM115DriverClient() { close_port(); }

bool DDSM115DriverClient::init_port(const std::string &port_name) {
  try {
    port_name_ = port_name;
    io_context_ = std::make_unique<boost::asio::io_context>();
    serial_port_ = std::make_unique<boost::asio::serial_port>(*io_context_, port_name_);
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    start_async_read();
    return true;
  } catch (const std::exception &e) {
    log(LogLevel::ERROR, "Failed to open port: " + std::string(e.what()));
    return false;
  }
}

void DDSM115DriverClient::close_port() {
  if (serial_port_ && serial_port_->is_open()) {
    serial_port_->close();
  }
  if (io_context_) {
    io_context_->stop();
  }
}

std::vector<uint8_t> DDSM115DriverClient::create_mode_command(uint8_t motor_id,
                                                              ControlLoopModes mode) {
  std::vector<uint8_t> data;
  data.reserve(10);
  data.push_back(motor_id);
  data.push_back(0xA0);
  data.push_back(0x00);
  data.push_back(0x00);
  data.push_back(0x00);
  data.push_back(0x00);
  data.push_back(0x00);
  data.push_back(0x00);
  data.push_back(0x00);
  data.push_back(static_cast<uint8_t>(mode));
  return data;
}

std::vector<uint8_t> DDSM115DriverClient::create_velocity_command(uint8_t motor_id,
                                                                  double rpm, bool brake) {
  std::vector<uint8_t> data;
  data.reserve(10);
  data.push_back(motor_id);
  data.push_back(0x64);
  uint16_t val_u16 = std::bit_cast<uint16_t>(
      static_cast<int16_t>(std::clamp(std::round(rpm), -330.0, 330.0)));
  data.push_back(static_cast<uint8_t>((val_u16 >> 8) & 0xFF));
  data.push_back(static_cast<uint8_t>(val_u16 & 0xFF));
  data.push_back(0x00);
  data.push_back(0x00);
  data.push_back(0x00);
  data.push_back(brake ? 0xFF : 0x00);
  data.push_back(0x00);
  data.push_back(calc_crc8_maxim(data));
  return data;
}

void DDSM115DriverClient::feed_data(const std::vector<uint8_t> &data) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  buffer_.insert(buffer_.end(), data.begin(), data.end());
  parse_buffer();
}

void DDSM115DriverClient::start_async_read() {
  if (!serial_port_ || !serial_port_->is_open()) return;
  serial_port_->async_read_some(
      boost::asio::buffer(read_buf_),
      [this](const boost::system::error_code &ec, std::size_t bytes_transferred) {
        if (!ec) {
          std::vector<uint8_t> data(read_buf_.begin(), read_buf_.begin() + bytes_transferred);
          feed_data(data);
          start_async_read();
        }
      });
}

void DDSM115DriverClient::parse_buffer() {
  while (buffer_.size() >= 10) {
    if (buffer_[1] == 0x64 || buffer_[1] == 0x00) {  // Feedback types
      std::vector<uint8_t> packet(buffer_.begin(), buffer_.begin() + 10);
      if (calc_crc8_maxim(packet) == packet[9]) {
        process_feedback_packet(packet);
        buffer_.erase(buffer_.begin(), buffer_.begin() + 10);
      } else {
        buffer_.erase(buffer_.begin());
      }
    } else {
      buffer_.erase(buffer_.begin());
    }
  }
}

void DDSM115DriverClient::process_feedback_packet(const std::vector<uint8_t> &packet) {
  if (feedback_callback_) {
    feedback_callback_(packet);
  }
}

void DDSM115DriverClient::log(LogLevel level, const std::string &message) {
  if (log_callback_) {
    log_callback_(level, message);
  }
}

}  // namespace ddsm115_ros2_driver
