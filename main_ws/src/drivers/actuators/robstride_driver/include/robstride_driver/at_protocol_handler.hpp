// Copyright 2026 Tatsukiyano
#ifndef ROBSTRIDE_DRIVER__AT_PROTOCOL_HANDLER_HPP_
#define ROBSTRIDE_DRIVER__AT_PROTOCOL_HANDLER_HPP_

#include <cmath>
#include <algorithm>
#include "robstride_driver/robstride_protocol.hpp"
#include "robstride_driver/at_protocol.hpp"

namespace robstride_driver
{

class AtProtocolHandler : public RobstrideProtocol
{
public:
  AtProtocolHandler(double vel_max, int max_delta)
  : vel_max_(vel_max), max_at_command_delta_(max_delta) {}

  std::vector<uint8_t> build_at_frame(
    uint8_t command_id,
    uint8_t motor_id,
    const std::vector<uint8_t> & data)
  {
    uint8_t byte0 = command_id << 3;
    uint8_t byte1 = 0x07;
    uint8_t byte2, byte3;
    if (motor_id == 0xFC) {
      byte2 = 0xEB;
      byte3 = 0xFC;
    } else {
      byte2 = 0xE8 | ((motor_id >> 5) & 0x07);
      byte3 = ((motor_id << 3) & 0xFF) | 4;
    }

    std::vector<uint8_t> frame = {
      0x41, 0x54, // 'A', 'T'
      byte0, byte1, byte2, byte3,
      static_cast<uint8_t>(data.size())
    };
    frame.insert(frame.end(), data.begin(), data.end());
    frame.push_back(0x0D); // '\r'
    frame.push_back(0x0A); // '\n'
    return frame;
  }

  std::vector<uint8_t> create_enable_command(uint8_t motor_id) override
  {
    // Command 3 (Enable)
    return build_at_frame(3, motor_id, {0, 0, 0, 0, 0, 0, 0, 0});
  }

  std::vector<uint8_t> create_disable_command(uint8_t motor_id) override
  {
    // Command 4 (Stop)
    return build_at_frame(4, motor_id, {0, 0, 0, 0, 0, 0, 0, 0});
  }

  std::vector<uint8_t> create_velocity_command(uint8_t motor_id, double velocity_rad_s) override
  {
    // Command 18 (Write Parameter)
    // Register 0x700A (Velocity Control) -> LSB 0x0A, MSB 0x70
    double clamped_vel = std::clamp(velocity_rad_s, -vel_max_, vel_max_);
    float val_f = static_cast<float>(clamped_vel);
    uint8_t val_bytes[4];
    std::memcpy(val_bytes, &val_f, 4);

    return build_at_frame(18, motor_id, {
        0x0A, 0x70, 0x00, 0x00,
        val_bytes[0], val_bytes[1], val_bytes[2], val_bytes[3]
      });
  }

  std::vector<uint8_t> create_id_set_command(uint8_t /*motor_id*/, uint8_t new_id) override
  {
    // Write register 0x00C4 using ID-specific broadcast formatting
    // Command 4 (Set ID) targets new_id via standard mask
    return build_at_frame(4, new_id, {0x00, 0xC4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
  }

  std::vector<uint8_t> create_mode_select_command(
    uint8_t motor_id,
    const std::string & mode) override
  {
    // Command 18 (Write Parameter)
    // Register 0x7005 (Control Mode) -> LSB 0x05, MSB 0x70
    uint8_t mode_value = (mode == "position") ? 1 : 2;
    return build_at_frame(18, motor_id, {
        0x05, 0x70, 0x00, 0x00,
        mode_value, 0x00, 0x00, 0x00
      });
  }

  DecodeResult decode_frame(const std::vector<uint8_t> & data) override
  {
    DecodeResult result;
    if (data.size() < 17) {
      result.error_msg = "Frame too short: " + std::to_string(data.size());
      return result;
    }
    if (data[0] != 0x41 || data[1] != 0x54) {
      result.error_msg = "Invalid AT header";
      return result;
    }

    // Decode Motor ID using inverse mask rule: Byte 2 (index 4) and Byte 3 (index 5)
    uint8_t byte2 = data[4];
    uint8_t byte3 = data[5];
    result.motor_id = ((byte2 & 0x07) << 5) | ((byte3 & 0xF8) >> 3);

    uint16_t pos_u = (data[7] << 8) | data[8];
    uint16_t vel_u = (data[9] << 8) | data[10];
    uint16_t tor_u = (data[11] << 8) | data[12];

    result.state.position = uint_to_float(pos_u, -12.57, 12.57);
    result.state.velocity = uint_to_float(vel_u, -50.0, 50.0);
    result.state.effort = uint_to_float(tor_u, -6.0, 6.0);
    result.success = true;

    return result;
  }

  std::string get_default_tx_topic() const override {return "/communication/tx";}
  std::string get_default_rx_topic() const override {return "/communication/rx";}

private:
  double uint_to_float(uint16_t value, double low, double high)
  {
    double span = high - low;
    return static_cast<double>(value) * span / 65535.0 + low;
  }
  double vel_max_;
  int max_at_command_delta_;
};

}  // namespace robstride_driver

#endif  // ROBSTRIDE_DRIVER__AT_PROTOCOL_HANDLER_HPP_
