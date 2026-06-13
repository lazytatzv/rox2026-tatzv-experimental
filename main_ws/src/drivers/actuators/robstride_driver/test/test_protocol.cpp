// Copyright 2026 Tatsukiyano
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <memory>

#include "robstride_driver/at_protocol_handler.hpp"
#include "robstride_driver/can_protocol_handler.hpp"

using namespace robstride_driver;

class ProtocolTest : public ::testing::Test {
protected:
  void SetUp() override {
    at_handler = std::make_unique<AtProtocolHandler>(50.0, 16383);
    can_handler = std::make_unique<CanProtocolHandler>();
  }
  std::unique_ptr<AtProtocolHandler> at_handler;
  std::unique_ptr<CanProtocolHandler> can_handler;
};

// --- AT Protocol Tests ---

TEST_F(ProtocolTest, AtEnableCommand) {
  auto frame = at_handler->create_enable_command(0x0C);
  ASSERT_EQ(frame.size(), 16);
  EXPECT_EQ(frame[0], 0x41); // 'A'
  EXPECT_EQ(frame[1], 0x54); // 'T'
  EXPECT_EQ(frame[5], 0x0C);
  EXPECT_EQ(frame[8], 0xC4); // REG_ADDR_MOTOR_ENABLE
}

TEST_F(ProtocolTest, AtVelocityCommand) {
  double velocity = 10.0; // rad/s
  auto frame = at_handler->create_velocity_command(0x0A, velocity);
  ASSERT_EQ(frame.size(), 16);
  EXPECT_EQ(frame[5], 0x0A);
  EXPECT_EQ(frame[8], 0x70); // REG_ADDR_VELOCITY_CTRL
  
  // Neutral is 0x7FFF. Positive velocity should be > 0x7FFF
  uint16_t at_val = (frame[13] << 8) | frame[14];
  EXPECT_GT(at_val, 0x7FFF);
}

TEST_F(ProtocolTest, AtDecodeFrame) {
  // Mock response: pos=0, vel=neutral, effort=0
  std::vector<uint8_t> mock_rx(16, 0);
  mock_rx[0] = 0x41; mock_rx[1] = 0x54;
  mock_rx[5] = 0x0B;
  
  // Pos=0x7FFF (~0 rad)
  mock_rx[7] = 0x7F; mock_rx[8] = 0xFF;
  // Vel=0x7FFF (~0 rad/s)
  mock_rx[9] = 0x7F; mock_rx[10] = 0xFF;
  
  auto result = at_handler->decode_frame(mock_rx);
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.motor_id, 0x0B);
  EXPECT_NEAR(result.state.position, 0.0, 0.1);
  EXPECT_NEAR(result.state.velocity, 0.0, 0.1);
}

// --- CAN Protocol Tests ---

TEST_F(ProtocolTest, CanVelocityCommand) {
  double velocity = 5.0;
  auto frame = can_handler->create_velocity_command(0x01, velocity);
  ASSERT_EQ(frame.size(), 16);
  EXPECT_EQ(frame[0], 0xAA);
  
  uint32_t id;
  std::memcpy(&id, &frame[1], 4);
  EXPECT_EQ(id, 0x401);
  
  int32_t raw_vel;
  std::memcpy(&raw_vel, &frame[8], 4);
  EXPECT_EQ(raw_vel, 5000);
}

TEST_F(ProtocolTest, CanDecodeFrame) {
  std::vector<uint8_t> mock_rx(16, 0);
  mock_rx[0] = 0xAA;
  uint32_t id = 0x502;
  std::memcpy(&mock_rx[1], &id, 4);
  
  int32_t pos = 1234; // 1.234 rad
  int16_t vel = 5678; // 5.678 rad/s
  std::memcpy(&mock_rx[8], &pos, 4);
  std::memcpy(&mock_rx[12], &vel, 2);
  
  auto result = can_handler->decode_frame(mock_rx);
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.motor_id, 0x02);
  EXPECT_NEAR(result.state.position, 1.234, 1e-3);
  EXPECT_NEAR(result.state.velocity, 5.678, 1e-3);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
