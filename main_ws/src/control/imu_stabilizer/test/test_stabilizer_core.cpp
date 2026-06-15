// Copyright 2026 Tatsukiyano
#include <gtest/gtest.h>
#include "imu_stabilizer/heading_stabilizer_core.hpp"

using namespace imu_stabilizer;

TEST(HeadingStabilizerCoreTest, TestLockActivation)
{
  HeadingStabilizerCore core;
  
  // No command -> Lock should activate
  core.updateCommand(0.0, 1.0); // current_yaw = 1.0
  EXPECT_TRUE(core.isLockActive());
  EXPECT_NEAR(core.getTargetYaw(), 1.0, 1e-6);

  // Command active -> Lock should deactivate
  core.updateCommand(0.5, 1.2);
  EXPECT_FALSE(core.isLockActive());
}

TEST(HeadingStabilizerCoreTest, TestNormalization)
{
  HeadingStabilizerCore core;
  
  core.updateCommand(0.0, 3.14); // Lock at PI
  // If we are at -3.14, error should be small (~0), not 2PI.
  // Note: normalization is internal, but we can verify result
  double out = core.compute(0.0, -3.14, 0.01);
  EXPECT_NEAR(out, 0.0, 0.5); 
}

TEST(HeadingStabilizerCoreTest, TestLPF)
{
  HeadingStabilizerConfig config;
  config.gyro_alpha = 0.5;
  HeadingStabilizerCore core(config);
  
  // First update
  double out1 = core.compute(1.0, 0.0, 0.01); // filtered = 0.5 * 1.0 + 0.5 * 0 = 0.5
  // Second update
  double out2 = core.compute(1.0, 0.0, 0.01); // filtered = 0.5 * 1.0 + 0.5 * 0.5 = 0.75
  
  // rate_error = target(0) - filtered
  // correction = P * rate_error = 0.5 * -0.75 = -0.375
  EXPECT_NEAR(out2, -0.375, 0.1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
