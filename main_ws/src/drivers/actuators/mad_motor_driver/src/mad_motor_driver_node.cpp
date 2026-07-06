#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>

using namespace std::chrono_literals;

class MadMotorDriver : public rclcpp::Node
{
public:
  MadMotorDriver() : Node("mad_motor_driver_node"), current_target_rpm_(0.0)
  {
    // Parameters
    this->declare_parameter("can_interface", "can0");
    this->declare_parameter("motor_id_left", 0x201);
    this->declare_parameter("motor_id_right", 0x202);
    this->declare_parameter("watchdog_timeout", 0.5);

    watchdog_timeout_ = this->get_parameter("watchdog_timeout").as_double();

    // Subscriptions
    cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/shooter/cmd_muxed", 10,
      std::bind(&MadMotorDriver::cmd_callback, this, std::placeholders::_1));

    // Watchdog Timer (checks if commands stopped coming)
    watchdog_timer_ = this->create_wall_timer(
      100ms, std::bind(&MadMotorDriver::watchdog_callback, this));

    // CAN Tx Timer (sends commands to CAN at 100Hz)
    can_tx_timer_ = this->create_wall_timer(
      10ms, std::bind(&MadMotorDriver::can_tx_callback, this));

    last_cmd_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "MAD Motor CAN Driver started. Awaiting RPM commands.");
  }

private:
  void cmd_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    current_target_rpm_ = msg->data;
    last_cmd_time_ = this->now();
  }

  void watchdog_callback()
  {
    auto now = this->now();
    double elapsed = (now - last_cmd_time_).seconds();

    if (elapsed > watchdog_timeout_ && current_target_rpm_ != 0.0) {
      RCLCPP_WARN(this->get_logger(), "Watchdog timeout! No commands received for %.2f s. Stopping motors.", elapsed);
      current_target_rpm_ = 0.0;
    }
  }

  void can_tx_callback()
  {
    // In a real implementation, write current_target_rpm_ to SocketCAN (e.g., using <linux/can.h>)
    // Example: STM32 CAN ID 0x201 for left motor, 0x202 for right motor
    
    // Convert RPM to integer bytes if required by STM32
    // int32_t target_rpm_int = static_cast<int32_t>(current_target_rpm_);
    
    // For now, we just simulate the CAN transmission output for debugging
    // RCLCPP_DEBUG(this->get_logger(), "Tx CAN: Left ID: 0x%03X, Right ID: 0x%03X, Target RPM: %.2f", 
    //   this->get_parameter("motor_id_left").as_int(),
    //   this->get_parameter("motor_id_right").as_int(),
    //   current_target_rpm_);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr can_tx_timer_;
  rclcpp::Time last_cmd_time_;
  
  double current_target_rpm_;
  double watchdog_timeout_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MadMotorDriver>());
  rclcpp::shutdown();
  return 0;
}
