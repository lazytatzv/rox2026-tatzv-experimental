// Copyright 2026 Tatsukiyano
#ifndef IMU_STABILIZER__HEADING_STABILIZER_CORE_HPP_
#define IMU_STABILIZER__HEADING_STABILIZER_CORE_HPP_

#include <cmath>
#include <algorithm>
#include "control_toolbox/pid.hpp"

namespace imu_stabilizer
{

struct HeadingStabilizerConfig
{
  double heading_p = 3.0;
  double heading_i = 0.5;
  double heading_d = 0.0;
  double rate_p = 0.5;
  double rate_i = 0.0;
  double rate_d = 0.05;
  double heading_limit = 1.0;
  double rate_limit = 0.5;
  double gyro_alpha = 0.3;
};

/**
 * @brief Pure logic class for heading stabilization.
 * Separated from ROS 2 to allow unit testing and reuse.
 */
class HeadingStabilizerCore
{
public:
  explicit HeadingStabilizerCore(const HeadingStabilizerConfig & config = HeadingStabilizerConfig());

  /**
   * @brief Update target yaw and lock state.
   * @param target_angular_z Raw command angular velocity.
   * @param current_yaw Current robot yaw.
   */
  void updateCommand(double target_angular_z, double current_yaw);

  /**
   * @brief Perform control calculation.
   * @param current_raw_rate Raw gyro angular velocity.
   * @param current_yaw Current robot yaw.
   * @param dt Time step in seconds.
   * @return Final commanded angular velocity.
   */
  double compute(double current_raw_rate, double current_yaw, double dt);

  // Accessors
  double getTargetYaw() const { return target_yaw_lock_; }
  bool isLockActive() const { return lock_active_; }
  void reset();

private:
  static double normalizeAngle(double angle);

  HeadingStabilizerConfig config_;
  control_toolbox::Pid pid_heading_;
  control_toolbox::Pid pid_rate_;

  double target_yaw_lock_ = 0.0;
  double filtered_rate_ = 0.0;
  bool lock_active_ = false;
};

} // namespace imu_stabilizer

#endif // IMU_STABILIZER__HEADING_STABILIZER_CORE_HPP_
