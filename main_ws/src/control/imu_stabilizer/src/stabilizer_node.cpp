// Copyright 2026 Tatsukiyano
#include "imu_stabilizer/stabilizer_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace imu_stabilizer
{

HeadingStabilizerNode::HeadingStabilizerNode(const rclcpp::NodeOptions & options)
: Node("heading_stabilizer", options),
  last_cmd_time_(0, 0, rcl_clock_type_t::RCL_ROS_TIME),
  last_control_time_(0, 0, rcl_clock_type_t::RCL_ROS_TIME)
{
  declare_parameters();

  // last_cmd_time_ will be properly set on first cmd_callback or in constructor body
  last_cmd_time_ = this->get_clock()->now();
  core_ = std::make_unique<HeadingStabilizerCore>(config_);

  sub_cmd_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel_in", 10, std::bind(&HeadingStabilizerNode::cmd_callback, this, std::placeholders::_1));
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10, std::bind(&HeadingStabilizerNode::odom_callback, this, std::placeholders::_1));
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", rclcpp::SensorDataQoS(), std::bind(&HeadingStabilizerNode::imu_callback, this, std::placeholders::_1));

  pub_cmd_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_out", 10);

  // Parameter callback for dynamic tuning
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&HeadingStabilizerNode::on_parameter_change, this, std::placeholders::_1));

  // Use Node Clock instead of Wall Timer to avoid TimeSource mismatch in simulation
  timer_ = this->create_timer(10ms, std::bind(&HeadingStabilizerNode::control_loop, this));

  RCLCPP_INFO(get_logger(), "Heading Stabilizer Component Ready (Config-Driven)");
}

void HeadingStabilizerNode::declare_parameters()
{
  this->declare_parameter("enable_lock", true);
  this->declare_parameter("use_odom_for_yaw", false);
  this->declare_parameter("lock_deadband", 0.01);
  this->declare_parameter("gyro_alpha", 0.3);
  this->declare_parameter("heading_pid.p", 3.0);
  this->declare_parameter("heading_pid.i", 0.5);
  this->declare_parameter("heading_pid.d", 0.0);
  this->declare_parameter("heading_pid.i_clamp", 1.0);
  this->declare_parameter("rate_pid.p", 0.5);
  this->declare_parameter("rate_pid.i", 0.0);
  this->declare_parameter("rate_pid.d", 0.05);
  this->declare_parameter("rate_pid.i_clamp", 0.5);
}

void HeadingStabilizerNode::update_config_from_params()
{
  config_.lock_deadband = this->get_parameter("lock_deadband").as_double();
  config_.gyro_alpha = this->get_parameter("gyro_alpha").as_double();
  config_.heading_p = this->get_parameter("heading_pid.p").as_double();
  config_.heading_i = this->get_parameter("heading_pid.i").as_double();
  config_.heading_d = this->get_parameter("heading_pid.d").as_double();
  config_.heading_limit = this->get_parameter("heading_pid.i_clamp").as_double();
  config_.rate_p = this->get_parameter("rate_pid.p").as_double();
  config_.rate_i = this->get_parameter("rate_pid.i").as_double();
  config_.rate_d = this->get_parameter("rate_pid.d").as_double();
  config_.rate_limit = this->get_parameter("rate_pid.i_clamp").as_double();
}

rcl_interfaces::msg::SetParametersResult HeadingStabilizerNode::on_parameter_change(
  const std::vector<rclcpp::Parameter> & /*params*/)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  update_config_from_params();
  if (core_) core_->setGains(config_);
  return result;
}

void HeadingStabilizerNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  current_raw_rate_ = msg->angular_velocity.z;
  if (!this->get_parameter("use_odom_for_yaw").as_bool()) {
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;
    // Simple yaw extraction assuming flat ground (roll/pitch approx 0)
    current_yaw_ = 2.0 * std::atan2(qz, qw);
  }
}

void HeadingStabilizerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (this->get_parameter("use_odom_for_yaw").as_bool()) {
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    current_yaw_ = 2.0 * std::atan2(qz, qw);
  }
}

void HeadingStabilizerNode::cmd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  last_cmd_ = *msg;
  last_cmd_time_ = this->get_clock()->now();
  core_->updateCommand(msg->twist.angular.z, current_yaw_);
}

void HeadingStabilizerNode::control_loop()
{
  auto now = this->get_clock()->now();
  
  if (last_control_time_.nanoseconds() == 0) {
    last_control_time_ = now;
    return;
  }
  
  double dt_s = (now - last_control_time_).seconds();
  last_control_time_ = now;
  
  if (dt_s <= 0.0) {
    dt_s = 0.01; // Fallback
  }

  if ((now - last_cmd_time_).seconds() > 0.5) {
    pub_cmd_->publish(geometry_msgs::msg::TwistStamped());
    return;
  }

  if (!this->get_parameter("enable_lock").as_bool()) {
    pub_cmd_->publish(last_cmd_);
    return;
  }

  double out_rate = core_->compute(current_raw_rate_, current_yaw_, dt_s);

  auto out_msg = last_cmd_;
  out_msg.header.stamp = now;
  out_msg.twist.angular.z = (core_->isLockActive()) ? out_rate : last_cmd_.twist.angular.z;

  pub_cmd_->publish(out_msg);
}

} // namespace imu_stabilizer

RCLCPP_COMPONENTS_REGISTER_NODE(imu_stabilizer::HeadingStabilizerNode)
