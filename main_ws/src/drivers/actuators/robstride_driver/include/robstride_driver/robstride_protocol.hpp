// Copyright 2026 Tatsukiyano
#ifndef ROBSTRIDE_DRIVER__ROBSTRIDE_PROTOCOL_HPP_
#define ROBSTRIDE_DRIVER__ROBSTRIDE_PROTOCOL_HPP_

#include <vector>
#include <cstdint>
#include <optional>
#include <string>

namespace robstride_driver
{

struct MotorState
{
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;
};

struct DecodeResult
{
  bool success = false;
  uint8_t motor_id = 0;
  MotorState state;
  std::string error_msg = "";
};

class RobstrideProtocol
{
public:
  virtual ~RobstrideProtocol() = default;

  virtual std::vector<uint8_t> create_enable_command(uint8_t motor_id) = 0;
  virtual std::vector<uint8_t> create_disable_command(uint8_t motor_id) = 0;
  virtual std::vector<uint8_t> create_velocity_command(uint8_t motor_id, double velocity_rad_s) = 0;
  virtual std::vector<uint8_t> create_id_set_command(uint8_t motor_id, uint8_t new_id) = 0;
  
  virtual DecodeResult decode_frame(const std::vector<uint8_t> & data) = 0;
  
  virtual std::string get_default_tx_topic() const = 0;
  virtual std::string get_default_rx_topic() const = 0;
};

}  // namespace robstride_driver

#endif  // ROBSTRIDE_DRIVER__ROBSTRIDE_PROTOCOL_HPP_
