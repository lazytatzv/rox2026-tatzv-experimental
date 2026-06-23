// Copyright 2026 Tatsukiyano
#ifndef ROBSTRIDE_DRIVER__ROBSTRIDE_SERIAL_DRIVER_HPP_
#define ROBSTRIDE_DRIVER__ROBSTRIDE_SERIAL_DRIVER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

namespace robstride_driver
{

struct RobstrideSerialConfig
{
  std::string usb_path{"/dev/ttyUSB0"};
  int serial_baud{921600};
};

class RobstrideSerialDriver
{
public:
  static constexpr std::size_t READ_BUFFER_SIZE = 256U;

  RobstrideSerialDriver();
  ~RobstrideSerialDriver();

  void open(const RobstrideSerialConfig & config);
  void close();
  bool is_open() const;

  void set_receive_callback(std::function<void(const std::vector<uint8_t> &)> callback);
  void send_raw(const std::vector<uint8_t> & data);
  void drain();

private:
  void start_async_read();
  void extract_frames_and_callback(std::vector<uint8_t> & buffer);

  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;
  std::thread io_thread_;

  mutable std::mutex serial_mutex_;
  std::array<uint8_t, READ_BUFFER_SIZE> read_buffer_{};
  std::vector<uint8_t> rx_bytes_;

  std::mutex callback_mutex_;
  std::function<void(const std::vector<uint8_t> &)> on_receive_;

  std::atomic<bool> running_{false};
};

}  // namespace robstride_driver

#endif  // ROBSTRIDE_DRIVER__ROBSTRIDE_SERIAL_DRIVER_HPP_
