// Copyright 2026 Tatsukiyano
#ifndef ROBSTRIDE_DRIVER__DDSM_PROTOCOL_HANDLER_HPP_
#define ROBSTRIDE_DRIVER__DDSM_PROTOCOL_HANDLER_HPP_

#include <cmath>
#include <algorithm>
#include "robstride_driver/robstride_protocol.hpp"

namespace robstride_driver
{

class DdsmProtocolHandler : public RobstrideProtocol
{
public:
  DdsmProtocolHandler() = default;

  std::vector<uint8_t> create_enable_command(uint8_t /*motor_id*/) override {
    return {}; // DDSM115 typically doesn't need explicit enable via velocity command
  }

  std::vector<uint8_t> create_disable_command(uint8_t /*motor_id*/) override {
    return {};
  }

  std::vector<uint8_t> create_velocity_command(uint8_t motor_id, double velocity_rad_s) override {
    double rpm = (velocity_rad_s * 60.0) / (2.0 * M_PI);
    int16_t rpm_i16 = static_cast<int16_t>(rpm);

    return {
      motor_id,
      0x64, // Command code for velocity
      static_cast<uint8_t>((rpm_i16 >> 8) & 0xFF),
      static_cast<uint8_t>(rpm_i16 & 0xFF),
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // Padding/Check if needed by protocol
    };
  }

  std::optional<std::pair<uint8_t, MotorState>> decode_frame(const std::vector<uint8_t> & data) override {
    if (data.size() < 10) return std::nullopt;

    uint8_t motor_id = data[0];
    MotorState state;
    // Basic placeholder parsing - DDSM feedback parsing can be complex
    state.position = 0.0;
    state.velocity = 0.0;
    state.effort = 0.0;

    return std::make_pair(motor_id, state);
  }

  std::string get_default_tx_topic() const override { return "/serial_write"; }
  std::string get_default_rx_topic() const override { return "/serial_read"; }
};

}  // namespace robstride_driver

#endif  // ROBSTRIDE_DRIVER__DDSM_PROTOCOL_HANDLER_HPP_
