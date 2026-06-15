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

  std::vector<uint8_t> create_enable_command(uint8_t motor_id) override
  {
    using namespace at_protocol;
    return {
      FRAME_HEADER_A, FRAME_HEADER_T, CMD_BASIC_CONFIG,
      DEFAULT_SOURCE_ID_HI, DEFAULT_SOURCE_ID_LO, motor_id,
      DATA_LEN_8_BYTES, 0x00, REG_ADDR_MOTOR_ENABLE,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      FRAME_FOOTER_CR, FRAME_FOOTER_LF
    };
  }

  std::vector<uint8_t> create_disable_command(uint8_t motor_id) override
  {
    using namespace at_protocol;
    return {
      FRAME_HEADER_A, FRAME_HEADER_T, CMD_BASIC_CONFIG,
      DEFAULT_SOURCE_ID_HI, DEFAULT_SOURCE_ID_LO, motor_id,
      DATA_LEN_8_BYTES, 0x00, REG_ADDR_MOTOR_ENABLE,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
      FRAME_FOOTER_CR, FRAME_FOOTER_LF
    };
  }

  std::vector<uint8_t> create_velocity_command(uint8_t motor_id, double velocity_rad_s) override
  {
    using namespace at_protocol;
    double clamped_vel = std::clamp(velocity_rad_s, -vel_max_, vel_max_);
    int delta = static_cast<int>(std::round((clamped_vel / vel_max_) * max_at_command_delta_));
    uint16_t at_value = NEUTRAL_VELOCITY_VALUE + delta;
    uint8_t direction_flag = (at_value == NEUTRAL_VELOCITY_VALUE) ? DIR_STOP : DIR_ROTATING;

    return {
      FRAME_HEADER_A, FRAME_HEADER_T, CMD_DATA_STREAMING,
      DEFAULT_SOURCE_ID_HI, DEFAULT_SOURCE_ID_LO, motor_id,
      DATA_LEN_8_BYTES, SPEED_CMD_INDICATOR, REG_ADDR_VELOCITY_CTRL,
      0x00, 0x00, CTRL_MODE_VELOCITY, direction_flag,
      static_cast<uint8_t>((at_value >> 8) & 0xFF),
      static_cast<uint8_t>(at_value & 0xFF),
      FRAME_FOOTER_CR, FRAME_FOOTER_LF
    };
  }

  std::vector<uint8_t> create_id_set_command(uint8_t motor_id, uint8_t new_id) override
  {
    using namespace at_protocol;
    // Format: AT [CMD] [SRC_HI] [SRC_LO] [ID] [LEN] [SUB] [REG] [DATA...] [CR] [LF]
    // Write register 0x03 (CAN ID)
    return {
      FRAME_HEADER_A, FRAME_HEADER_T, CMD_BASIC_CONFIG,
      DEFAULT_SOURCE_ID_HI, DEFAULT_SOURCE_ID_LO, motor_id,
      DATA_LEN_8_BYTES, 0x00, REG_ADDR_CAN_ID,
      0x00, 0x00, 0x00, new_id, 0x00, 0x00, // Data (4 bytes) + padding (2 bytes)
      FRAME_FOOTER_CR, FRAME_FOOTER_LF
    };
  }

  DecodeResult decode_frame(const std::vector<uint8_t> & data) override
  {
    using namespace at_protocol;
    DecodeResult result;
    if (data.size() < 16) {
      result.error_msg = "Frame too short: " + std::to_string(data.size());
      return result;
    }
    if (data[0] != FRAME_HEADER_A || data[1] != FRAME_HEADER_T) {
      result.error_msg = "Invalid AT header";
      return result;
    }

    result.motor_id = data[5];
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
