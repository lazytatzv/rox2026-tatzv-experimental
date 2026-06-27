// Copyright 2026 Tatsukiyano
#ifndef ROBSTRIDE_DRIVER__PRIVATE_PROTOCOL_HANDLER_HPP_
#define ROBSTRIDE_DRIVER__PRIVATE_PROTOCOL_HANDLER_HPP_

#include <cstring>
#include <vector>
#include <map>
#include <string>
#include "robstride_driver/robstride_protocol.hpp"

namespace robstride_driver
{

class PrivateProtocolHandler : public RobstrideProtocol
{
public:
  PrivateProtocolHandler(uint8_t master_id = 0x00, float kp = 6.0f, float ki = 0.02f, float limit_cur = 5.0f)
  : master_id_(master_id), default_kp_(kp), default_ki_(ki), default_limit_cur_(limit_cur)
  {}

  std::vector<uint8_t> create_enable_command(uint8_t motor_id) override
  {
    return build_frame(motor_id, 3, {});
  }

  std::vector<uint8_t> create_disable_command(uint8_t motor_id) override
  {
    return build_frame(motor_id, 4, {});
  }

  std::vector<uint8_t> create_velocity_command(uint8_t motor_id, double velocity_rad_s) override
  {
    return build_param_write_float(motor_id, 0x700A, static_cast<float>(velocity_rad_s));
  }

  std::vector<uint8_t> create_id_set_command(uint8_t motor_id, uint8_t new_id) override
  {
    (void)motor_id;
    (void)new_id;
    return {}; // Unimplemented for now
  }

  std::vector<uint8_t> create_mode_select_command(
    uint8_t motor_id,
    const std::string & mode) override
  {
    std::vector<uint8_t> frames;
    if (mode == "velocity") {
      // 1. Set Run Mode = 2 (Velocity)
      auto f1 = build_param_write_uint8(motor_id, 0x7005, 2);
      frames.insert(frames.end(), f1.begin(), f1.end());
      
      // 2. Set Kp
      auto f2 = build_param_write_float(motor_id, 0x701F, default_kp_);
      frames.insert(frames.end(), f2.begin(), f2.end());
      
      // 3. Set Ki
      auto f3 = build_param_write_float(motor_id, 0x7020, default_ki_);
      frames.insert(frames.end(), f3.begin(), f3.end());
      
      // 4. Set Current Limit
      auto f4 = build_param_write_float(motor_id, 0x7018, default_limit_cur_);
      frames.insert(frames.end(), f4.begin(), f4.end());
    }
    return frames;
  }

  DecodeResult decode_frame(const std::vector<uint8_t> & data) override
  {
    DecodeResult result;
    if (data.size() < 16 || data[0] != 0xAA) {
      result.error_msg = "Invalid private frame";
      return result;
    }

    uint32_t id;
    std::memcpy(&id, &data[1], 4);
    
    // In private protocol response, mode is 2 (motor feedback)
    uint8_t mode = (id >> 24) & 0x1F;
    if (mode != 2) {
      result.error_msg = "Not a feedback frame";
      return result;
    }

    // Extract motor ID from the host ID field of feedback (bit 8-15)
    // Actually wait, let's check manual 4.1.3:
    // Response frame ID: bit 23~8: bit 15~8 is CAN ID of the current motor.
    result.motor_id = static_cast<uint8_t>((id >> 8) & 0xFF);

    uint8_t angle_high = data[8];
    uint8_t angle_low = data[9];
    uint8_t vel_high = data[10];
    uint8_t vel_low = data[11];
    uint8_t tor_high = data[12];
    uint8_t tor_low = data[13];
    
    // Byte0~1: Current Angle [0~65535] -> (-4pi ~ 4pi)
    uint16_t angle_raw = (angle_high << 8) | angle_low;
    result.state.position = (static_cast<double>(angle_raw) / 65535.0) * 8.0 * 3.1415926535 - 4.0 * 3.1415926535;

    // Byte2~3: Current velocity [0~65535] -> (-50 ~ 50 rad/s)
    uint16_t vel_raw = (vel_high << 8) | vel_low;
    result.state.velocity = (static_cast<double>(vel_raw) / 65535.0) * 100.0 - 50.0;

    // Byte4~5: Current torque [0~65535] -> (-6 ~ 6 Nm)
    uint16_t tor_raw = (tor_high << 8) | tor_low;
    result.state.effort = (static_cast<double>(tor_raw) / 65535.0) * 12.0 - 6.0;

    result.success = true;
    return result;
  }

  std::string get_default_tx_topic() const override {return "/communication/tx";}
  std::string get_default_rx_topic() const override {return "/communication/rx";}

private:
  uint8_t master_id_;
  float default_kp_;
  float default_ki_;
  float default_limit_cur_;

  std::vector<uint8_t> build_frame(uint8_t motor_id, uint8_t mode, const std::vector<uint8_t>& payload)
  {
    std::vector<uint8_t> frame(16, 0);
    frame[0] = 0xAA;
    
    uint32_t ext_id = (static_cast<uint32_t>(mode) << 24) | (static_cast<uint32_t>(master_id_) << 8) | motor_id;
    std::memcpy(&frame[1], &ext_id, 4);

    frame[5] = 0x01; // Extended ID
    frame[6] = 0x00; // Data frame
    frame[7] = 0x08; // DLC always 8
    
    for (size_t i = 0; i < payload.size() && i < 8; i++) {
      frame[8 + i] = payload[i];
    }
    frame[15] = 0x55;
    return frame;
  }

  std::vector<uint8_t> build_param_write_uint8(uint8_t motor_id, uint16_t index, uint8_t value)
  {
    std::vector<uint8_t> payload(8, 0);
    std::memcpy(&payload[0], &index, 2);
    payload[4] = value;
    return build_frame(motor_id, 0x12, payload);
  }

  std::vector<uint8_t> build_param_write_float(uint8_t motor_id, uint16_t index, float value)
  {
    std::vector<uint8_t> payload(8, 0);
    std::memcpy(&payload[0], &index, 2);
    std::memcpy(&payload[4], &value, 4);
    return build_frame(motor_id, 0x12, payload);
  }
};

}  // namespace robstride_driver

#endif  // ROBSTRIDE_DRIVER__PRIVATE_PROTOCOL_HANDLER_HPP_
