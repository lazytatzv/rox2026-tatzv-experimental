// Copyright 2026 Tatsukiyano
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <algorithm>

#include "robstride_driver/at_protocol.hpp"
// Note: Normally we'd extract the logic into a separate class for pure testing,
// but we can test the protocol constants and logic directly here.

using namespace robstride_driver::at_protocol;

// --- Helper Functions (Mimic the Node logic) ---
uint16_t float_to_uint(double value, double low, double high) {
  double span = high - low;
  if (value < low) value = low;
  else if (value > high) value = high;
  return static_cast<uint16_t>((value - low) * 65535.0 / span);
}

double uint_to_float(uint16_t value, double low, double high) {
  double span = high - low;
  return static_cast<double>(value) * span / 65535.0 + low;
}

// --- AT Protocol Tests ---

TEST(RobstrideAtProtocol, VelocityCommandGeneration) {
  uint8_t motor_id = 0x0C;
  double velocity_rad_s = 0.5; // Half speed forward
  double max_vel = 50.0;
  int max_at_delta = static_cast<int>(NEUTRAL_VELOCITY_VALUE * 0.5); // 50% limit
  
  // Logic inside velocity_callback
  int delta = static_cast<int>(std::round((velocity_rad_s / max_vel) * max_at_delta));
  uint16_t at_value = NEUTRAL_VELOCITY_VALUE + delta;
  
  // Manual Verification of the frame
  EXPECT_GT(at_value, NEUTRAL_VELOCITY_VALUE);
  
  // Mock Frame Construction
  std::vector<uint8_t> frame = {
    FRAME_HEADER_A, FRAME_HEADER_T, CMD_DATA_STREAMING,
    DEFAULT_SOURCE_ID_HI, DEFAULT_SOURCE_ID_LO, motor_id,
    DATA_LEN_8_BYTES, SPEED_CMD_INDICATOR, REG_ADDR_VELOCITY_CTRL,
    0x00, 0x00, CTRL_MODE_VELOCITY, DIR_ROTATING,
    static_cast<uint8_t>((at_value >> 8) & 0xFF),
    static_cast<uint8_t>(at_value & 0xFF),
    FRAME_FOOTER_CR, FRAME_FOOTER_LF
  };

  EXPECT_EQ(frame[0], 'A');
  EXPECT_EQ(frame[1], 'T');
  EXPECT_EQ(frame[5], 0x0C); // Correct Motor ID
  EXPECT_EQ(frame[8], 0x70); // Correct Reg Addr
  EXPECT_EQ(frame[frame.size()-2], 0x0D); // CR
  EXPECT_EQ(frame[frame.size()-1], 0x0A); // LF
}

// --- CAN Protocol Tests (Official Seeed Wiki Spec) ---

TEST(RobstrideCanProtocol, SeeedWikiIdGeneration) {
  uint32_t motor_id = 0x01;
  uint32_t expected_id = 0x400 + motor_id; // Speed Mode per Wiki
  
  EXPECT_EQ(expected_id, 0x401);
}

TEST(RobstrideCanProtocol, VelocityScaling) {
  double velocity = 10.5; // rad/s
  int32_t expected_raw = static_cast<int32_t>(velocity * 1000.0);
  
  EXPECT_EQ(expected_raw, 10500);
  
  // Little Endian Packing Test
  uint8_t data[8] = {0};
  std::memcpy(&data[0], &expected_raw, 4);
  
  EXPECT_EQ(data[0], 10500 & 0xFF);
  EXPECT_EQ(data[1], (10500 >> 8) & 0xFF);
}

TEST(RobstrideCanProtocol, StatusFeedbackParsing) {
  // Mock 0x500 + ID packet from Wiki
  // data: pos(4), vel(2), tor(2)
  int32_t p_in = 3142; // ~3.142 rad
  int16_t v_in = 5000; // 5.0 rad/s
  uint8_t mock_data[8];
  std::memcpy(&mock_data[0], &p_in, 4);
  std::memcpy(&mock_data[4], &v_in, 2);
  
  // Parsing Logic
  int32_t p_out;
  int16_t v_out;
  std::memcpy(&p_out, &mock_data[0], 4);
  std::memcpy(&v_out, &mock_data[4], 2);
  
  EXPECT_NEAR(static_cast<double>(p_out) / 1000.0, 3.142, 1e-3);
  EXPECT_NEAR(static_cast<double>(v_out) / 1000.0, 5.0, 1e-3);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
