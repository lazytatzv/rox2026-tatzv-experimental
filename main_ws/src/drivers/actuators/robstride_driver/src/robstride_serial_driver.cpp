// Copyright 2026 Tatsukiyano
#include "robstride_driver/robstride_serial_driver.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <termios.h>

namespace robstride_driver
{

namespace
{

speed_t to_posix_baud(int baud)
{
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
#if defined(B460800)
    case 460800:
      return B460800;
#endif
#if defined(B500000)
    case 500000:
      return B500000;
#endif
#if defined(B576000)
    case 576000:
      return B576000;
#endif
#if defined(B921600)
    case 921600:
      return B921600;
#endif
#if defined(B1000000)
    case 1000000:
      return B1000000;
#endif
#if defined(B1500000)
    case 1500000:
      return B1500000;
#endif
#if defined(B2000000)
    case 2000000:
      return B2000000;
#endif
    default:
      throw std::invalid_argument("Unsupported serial_baud: " + std::to_string(baud));
  }
}

void configure_serial_port(int fd, int baud)
{
  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd, &tty) != 0) {
    throw std::system_error(errno, std::generic_category(), "tcgetattr failed");
  }

  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
#if defined(CRTSCTS)
  tty.c_cflag &= ~CRTSCTS;
#endif
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  const auto speed = to_posix_baud(baud);
  if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
    throw std::system_error(errno, std::generic_category(), "cfset*speed failed");
  }

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    throw std::system_error(errno, std::generic_category(), "tcsetattr failed");
  }

  if (tcflush(fd, TCIOFLUSH) != 0) {
    throw std::system_error(errno, std::generic_category(), "tcflush failed");
  }
}

}  // namespace

RobstrideSerialDriver::RobstrideSerialDriver()
: serial_port_(io_context_)
{
}

RobstrideSerialDriver::~RobstrideSerialDriver()
{
  close();
}

void RobstrideSerialDriver::open(const RobstrideSerialConfig & config)
{
  try {
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (serial_port_.is_open()) {
        throw std::runtime_error("Serial port already open");
      }

      serial_port_.open(config.usb_path);
      configure_serial_port(serial_port_.native_handle(), config.serial_baud);
    }

    // 1. シリアルポート開通コマンド: AT+AT\r\n
    std::vector<uint8_t> open_cmd = {0x41, 0x54, 0x2B, 0x41, 0x54, 0x0D, 0x0A};
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      boost::asio::write(serial_port_, boost::asio::buffer(open_cmd));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 2. デバイス検出コマンド: AT\x00\x07\xe8\x44\x01\x00\r\n
    std::vector<uint8_t> detect_cmd = {0x41, 0x54, 0x00, 0x07, 0xE8, 0x44, 0x01, 0x00, 0x0D, 0x0A};
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      boost::asio::write(serial_port_, boost::asio::buffer(detect_cmd));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    running_.store(true);
    start_async_read();
    io_thread_ = std::thread([this]() {io_context_.run();});
  } catch (...) {
    boost::system::error_code ec;
    serial_port_.cancel(ec);
    serial_port_.close(ec);
    running_.store(false);
    throw;
  }
}

void RobstrideSerialDriver::close()
{
  running_.store(false);
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    boost::system::error_code ec;
    serial_port_.cancel(ec);
    serial_port_.close(ec);
  }
  io_context_.stop();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  io_context_.restart();
}

bool RobstrideSerialDriver::is_open() const
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  return serial_port_.is_open();
}

void RobstrideSerialDriver::set_receive_callback(
  std::function<void(const std::vector<uint8_t> &)> callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  on_receive_ = std::move(callback);
}

void RobstrideSerialDriver::send_raw(const std::vector<uint8_t> & data)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (!serial_port_.is_open()) {
    throw std::runtime_error("Serial port is not open");
  }
  boost::asio::write(serial_port_, boost::asio::buffer(data));
}

void RobstrideSerialDriver::start_async_read()
{
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_port_.is_open()) {
      return;
    }
  }

  serial_port_.async_read_some(
    boost::asio::buffer(read_buffer_),
    [this](const boost::system::error_code & ec, std::size_t bytes_transferred) {
      if (ec) {
        if (ec == boost::asio::error::operation_aborted) {
          return;
        }
        if (running_.load() && is_open()) {
          start_async_read();
        }
        return;
      }

      {
        std::lock_guard<std::mutex> serial_lock(serial_mutex_);
        rx_bytes_.insert(
          rx_bytes_.end(),
          read_buffer_.begin(),
          read_buffer_.begin() + static_cast<std::ptrdiff_t>(bytes_transferred));

        extract_frames_and_callback(rx_bytes_);
      }

      if (running_.load() && is_open()) {
        start_async_read();
      }
    });
}

void RobstrideSerialDriver::extract_frames_and_callback(std::vector<uint8_t> & buffer)
{
  while (buffer.size() >= 17) {
    auto start_it = buffer.end();
    for (auto i = buffer.begin(); i != buffer.end() - 1; ++i) {
      if (*i == 0x41 && *(i + 1) == 0x54) { // 'A', 'T'
        start_it = i;
        break;
      }
    }

    if (start_it == buffer.end()) {
      if (buffer.back() == 0x41) {
        buffer.erase(buffer.begin(), buffer.end() - 1);
      } else {
        buffer.clear();
      }
      break;
    }

    if (start_it != buffer.begin()) {
      buffer.erase(buffer.begin(), start_it);
    }

    if (buffer.size() < 17) {
      break;
    }

    if (buffer[15] == 0x0D && buffer[16] == 0x0A) { // \r\n
      std::vector<uint8_t> frame(buffer.begin(), buffer.begin() + 17);
      buffer.erase(buffer.begin(), buffer.begin() + 17);

      std::function<void(const std::vector<uint8_t> &)> callback;
      {
        std::lock_guard<std::mutex> callback_lock(callback_mutex_);
        callback = on_receive_;
      }
      if (callback) {
        callback(frame);
      }
    } else {
      buffer.erase(buffer.begin());
    }
  }
}

}  // namespace robstride_driver
