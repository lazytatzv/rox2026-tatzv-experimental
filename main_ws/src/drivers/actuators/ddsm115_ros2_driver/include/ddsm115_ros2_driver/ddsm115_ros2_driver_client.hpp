// Copyright 2026 Tatsukiyano
#ifndef DDSM115_ROS2_DRIVER_DDSM115_ROS2_DRIVER_CLIENT_HPP_
#define DDSM115_ROS2_DRIVER_DDSM115_ROS2_DRIVER_CLIENT_HPP_

#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace ddsm115_ros2_driver {

enum class LogLevel { DEBUG, INFO, WARN, ERROR };
enum class ControlLoopModes {
  MODE_VELOCITY = 0x02,
  MODE_POSITION = 0x01,
  MODE_MIT = 0x00
};

using FeedbackCallback = std::function<void(const std::vector<uint8_t> &)>;
using LogCallback = std::function<void(LogLevel, const std::string &)>;

class DDSM115DriverClient {
 public:
  DDSM115DriverClient(FeedbackCallback feedback_callback,
                      LogCallback log_callback);
  ~DDSM115DriverClient();

  bool init_port(const std::string &port_name);
  void close_port();

  // Motor control functions
  std::vector<uint8_t> create_mode_command(uint8_t motor_id,
                                           ControlLoopModes mode);
  std::vector<uint8_t> create_velocity_command(uint8_t motor_id, double rpm,
                                               bool brake = false);

  // External data injection for parsing
  void feed_data(const std::vector<uint8_t> &data);

 private:
  // Helper functions
  void start_async_read();
  void parse_buffer();
  void process_feedback_packet(const std::vector<uint8_t> &packet);

  std::string port_name_;
  int baud_rate_;
  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<boost::asio::serial_port> serial_port_;
  std::vector<uint8_t> buffer_;
  std::array<uint8_t, 1024> read_buf_;
  std::mutex buffer_mutex_;

  FeedbackCallback feedback_callback_;
  LogCallback log_callback_;

  void log(LogLevel level, const std::string &message);
};
}  // namespace ddsm115_ros2_driver
#endif  // DDSM115_ROS2_DRIVER_DDSM115_ROS2_DRIVER_CLIENT_HPP_
