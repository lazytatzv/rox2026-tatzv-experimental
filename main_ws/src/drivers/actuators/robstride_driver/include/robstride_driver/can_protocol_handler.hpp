// Copyright 2026 Tatsukiyano
#ifndef ROBSTRIDE_DRIVER__CAN_PROTOCOL_HANDLER_HPP_
#define ROBSTRIDE_DRIVER__CAN_PROTOCOL_HANDLER_HPP_

#include <cstring>
#include "robstride_driver/robstride_protocol.hpp"

namespace robstride_driver
{

class CanProtocolHandler : public RobstrideProtocol
{
public:
  CanProtocolHandler() = default;

  std::vector<uint8_t> create_enable_command(uint8_t /*motor_id*/) override {
    // CAN mode might have different enable logic or not needed depending on setup
    return {}; 
  }

  std::vector<uint8_t> create_disable_command(uint8_t /*motor_id*/) override {
    return {};
  }

  std::vector<uint8_t> create_velocity_command(uint8_t motor_id, double velocity_rad_s) override {
    // Standard Seeed Frame: [AA] [ID(4)] [EXT] [REMOTE] [DLC] [DATA(8)] [55]
    std::vector<uint8_t> frame(16, 0);
    frame[0] = 0xAA;
    uint32_t id = 0x400 + motor_id;
    std::memcpy(&frame[1], &id, 4);
    frame[5] = 0x00; // Standard
    frame[6] = 0x00; // Data frame
    frame[7] = 0x08; // DLC
    
    int32_t raw_vel = static_cast<int32_t>(velocity_rad_s * 1000.0);
    std::memcpy(&frame[8], &raw_vel, 4);
    frame[15] = 0x55;
    return frame;
  }

  std::vector<uint8_t> create_id_set_command(uint8_t motor_id, uint8_t new_id) override {
    // RobStride/CyberGear Standard CAN ID Change (Command 18 / 0x12)
    // Structure: [AA] [ID(4)] [EXT] [REMOTE] [DLC] [DATA(8)] [55]
    std::vector<uint8_t> frame(16, 0);
    frame[0] = 0xAA;
    
    // Command 18: (0x12 << 24) | (TargetID << 16) | (HostID << 8)
    uint32_t ext_id = (0x12 << 24) | (static_cast<uint32_t>(motor_id) << 16) | (0xFE << 8);
    std::memcpy(&frame[1], &ext_id, 4);
    
    frame[5] = 0x01; // Extended ID
    frame[6] = 0x00; // Data frame
    frame[7] = 0x08; // DLC
    frame[8] = new_id; // New ID in Data[0]
    // Remaining data is 0
    
    frame[15] = 0x55;
    return frame;
  }

  DecodeResult decode_frame(const std::vector<uint8_t> & data) override {
    DecodeResult result;
    if (data.size() < 16) {
      result.error_msg = "CAN frame too short: " + std::to_string(data.size());
      return result;
    }
    if (data[0] != 0xAA) {
      result.error_msg = "Invalid CAN header (Expected 0xAA)";
      return result;
    }

    uint32_t id;
    std::memcpy(&id, &data[1], 4);
    if ((id & 0xF00) != 0x500) {
      result.error_msg = "Invalid CAN ID range (Expected 0x5XX)";
      return result;
    }

    result.motor_id = static_cast<uint8_t>(id & 0xFF);

    int32_t pos_raw;
    int16_t vel_raw;
    int16_t tor_raw;
    std::memcpy(&pos_raw, &data[8], 4);
    std::memcpy(&vel_raw, &data[12], 2);
    std::memcpy(&tor_raw, &data[14], 2);

    result.state.position = static_cast<double>(pos_raw) / 1000.0;
    result.state.velocity = static_cast<double>(vel_raw) / 1000.0;
    result.state.effort = static_cast<double>(tor_raw) / 1000.0;
    result.success = true;

    return result;
  }

  std::string get_default_tx_topic() const override { return "/communication/tx"; }
  std::string get_default_rx_topic() const override { return "/communication/rx"; }
};

}  // namespace robstride_driver

#endif  // ROBSTRIDE_DRIVER__CAN_PROTOCOL_HANDLER_HPP_
