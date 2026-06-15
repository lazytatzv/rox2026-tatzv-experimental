// Copyright 2026 Tatsukiyano
#include "imu_stabilizer/heading_stabilizer_core.hpp"

namespace imu_stabilizer
{

HeadingStabilizerCore::HeadingStabilizerCore(const HeadingStabilizerConfig & config)
: config_(config),
  pid_heading_(0.0, 0.0, 0.0, 0.0, 0.0, control_toolbox::AntiWindupStrategy()),
  pid_rate_(0.0, 0.0, 0.0, 0.0, 0.0, control_toolbox::AntiWindupStrategy())
{
  control_toolbox::AntiWindupStrategy aw_strat;
  aw_strat.type = control_toolbox::AntiWindupStrategy::CONDITIONAL_INTEGRATION;
  
  aw_strat.i_max = config_.heading_limit;
  aw_strat.i_min = -config_.heading_limit;
  pid_heading_.initialize(config_.heading_p, config_.heading_i, config_.heading_d, 
                          config_.heading_limit, -config_.heading_limit, aw_strat);
  
  aw_strat.i_max = config_.rate_limit;
  aw_strat.i_min = -config_.rate_limit;
  pid_rate_.initialize(config_.rate_p, config_.rate_i, config_.rate_d, 
                       config_.rate_limit, -config_.rate_limit, aw_strat);
}

void HeadingStabilizerCore::updateCommand(double target_angular_z, double current_yaw)
{
  if (std::abs(target_angular_z) < 0.001) {
    if (!lock_active_) {
      target_yaw_lock_ = current_yaw;
      lock_active_ = true;
    }
  } else {
    lock_active_ = false;
  }
}

double HeadingStabilizerCore::compute(double current_raw_rate, double current_yaw, double dt)
{
  // LPF for gyro
  filtered_rate_ = (config_.gyro_alpha * current_raw_rate) + ((1.0 - config_.gyro_alpha) * filtered_rate_);

  double target_rate = 0.0; // Assume stabilized mode or external cmd
  if (lock_active_) {
    double yaw_error = normalizeAngle(target_yaw_lock_ - current_yaw);
    target_rate = pid_heading_.compute_command(yaw_error, dt);
  }

  double rate_error = target_rate - filtered_rate_;
  double final_correction = pid_rate_.compute_command(rate_error, dt);

  return target_rate + final_correction;
}

void HeadingStabilizerCore::reset()
{
  pid_heading_.reset();
  pid_rate_.reset();
  filtered_rate_ = 0.0;
  lock_active_ = false;
}

double HeadingStabilizerCore::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

} // namespace imu_stabilizer
