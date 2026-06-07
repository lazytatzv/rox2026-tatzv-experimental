// Copyright 2026 Tatsukiyano
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include "mecanum_kinematics/kinematics.hpp"

// Official Constants from mecanum_rc.py
static constexpr double L = 0.12;
static constexpr double W = 0.10;
static constexpr double R = 0.05;
static constexpr double K = L + W;

TEST(MecanumKinematicsTest, OfficialFormulaValidation) {
  // Pattern 1: Pure Forward (vx=1.0)
  // Official: fl=1.0, fr=1.0, rl=1.0, rr=1.0 (before dividing by R)
  auto out = mecanum_kinematics::compute_wheel_speeds(1.0, 0.0, 0.0, L, W, R);
  EXPECT_NEAR(out[0], 1.0 / R, 1e-9);
  EXPECT_NEAR(out[1], 1.0 / R, 1e-9);
  EXPECT_NEAR(out[2], 1.0 / R, 1e-9);
  EXPECT_NEAR(out[3], 1.0 / R, 1e-9);

  // Pattern 2: Pure Strafe Left (vy=1.0)
  // Official: fl = -1.0, fr = 1.0, rl = 1.0, rr = -1.0
  out = mecanum_kinematics::compute_wheel_speeds(0.0, 1.0, 0.0, L, W, R);
  EXPECT_NEAR(out[0], -1.0 / R, 1e-9);
  EXPECT_NEAR(out[1], 1.0 / R, 1e-9);
  EXPECT_NEAR(out[2], 1.0 / R, 1e-9);
  EXPECT_NEAR(out[3], -1.0 / R, 1e-9);

  // Pattern 3: Pure CCW Rotation (omega=1.0)
  // Official: fl = -K, fr = K, rl = -K, rr = K
  out = mecanum_kinematics::compute_wheel_speeds(0.0, 0.0, 1.0, L, W, R);
  EXPECT_NEAR(out[0], -K / R, 1e-9);
  EXPECT_NEAR(out[1], K / R, 1e-9);
  EXPECT_NEAR(out[2], -K / R, 1e-9);
  EXPECT_NEAR(out[3], K / R, 1e-9);
}

TEST(MecanumKinematicsTest, ForwardKinematicsLoopback) {
  // Test if compute_body_twist correctly reverses compute_wheel_speeds
  double vx = 0.5, vy = -0.3, omega = 0.8;
  auto speeds = mecanum_kinematics::compute_wheel_speeds(vx, vy, omega, L, W, R);
  auto twist = mecanum_kinematics::compute_body_twist(speeds, L, W, R);

  EXPECT_NEAR(twist[0], vx, 1e-9);
  EXPECT_NEAR(twist[1], vy, 1e-9);
  EXPECT_NEAR(twist[2], omega, 1e-9);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
