#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <can_msgs/msg/frame.hpp>
#include <chrono>
#include <cstring>

using namespace std::chrono_literals;

class MadMotorDriver : public rclcpp::Node
{
public:
  MadMotorDriver() : Node("mad_motor_driver_node"), current_target_rpm_(0.0)
  {
    this->declare_parameter("motor_id_left", 0x201);
    this->declare_parameter("motor_id_right", 0x202);
    this->declare_parameter("limit_switch_id", 0x200);
    this->declare_parameter("watchdog_timeout", 0.5);

    watchdog_timeout_ = this->get_parameter("watchdog_timeout").as_double();

    // 1. Publisher for Limit Switch (Bool)
    limit_switch_pub_ = this->create_publisher<std_msgs::msg::Bool>("/shooter/limit_switch", 10);

    // 2. Publisher to ROS2 SocketCAN node
    can_tx_pub_ = this->create_publisher<can_msgs::msg::Frame>("/to_can", 10);

    // 3. Subscriber for Target RPM from Strategy/Teleop
    cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/shooter/cmd_muxed", 10,
      std::bind(&MadMotorDriver::cmd_callback, this, std::placeholders::_1));

    // 4. Subscriber from ROS2 SocketCAN node (Limit Switch RX)
    can_rx_sub_ = this->create_subscription<can_msgs::msg::Frame>(
      "/from_can", 10,
      std::bind(&MadMotorDriver::can_rx_callback, this, std::placeholders::_1));

    // 5. Watchdog Timer (stops motors if commands are lost)
    watchdog_timer_ = this->create_wall_timer(
      100ms, std::bind(&MadMotorDriver::watchdog_callback, this));

    // 6. CAN Tx Timer (sends Target RPM commands via /to_can at 100Hz)
    can_tx_timer_ = this->create_wall_timer(
      10ms, std::bind(&MadMotorDriver::can_tx_callback, this));

    last_cmd_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "MAD Motor CAN Driver started. Using ros2_socketcan (can_msgs).");
  }

private:
  void cmd_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    current_target_rpm_ = msg->data;
    last_cmd_time_ = this->now();
  }

  void can_rx_callback(const can_msgs::msg::Frame::SharedPtr msg)
  {
    uint32_t limit_switch_id = this->get_parameter("limit_switch_id").as_int();

    if (msg->id == limit_switch_id && msg->dlc >= 1) {
      std_msgs::msg::Bool sw_msg;
      sw_msg.data = (msg->data[0] == 1);
      limit_switch_pub_->publish(sw_msg);
    }
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
    can_msgs::msg::Frame frame_left;
    can_msgs::msg::Frame frame_right;

    frame_left.is_rtr = false;
    frame_left.is_extended = false;
    frame_left.is_error = false;
    frame_left.dlc = 4;

    frame_right.is_rtr = false;
    frame_right.is_extended = false;
    frame_right.is_error = false;
    frame_right.dlc = 4;

    // Convert double to float to match STM32 expectation
    float rpm_f32 = static_cast<float>(current_target_rpm_);
    std::memcpy(frame_left.data.data(), &rpm_f32, sizeof(float));
    std::memcpy(frame_right.data.data(), &rpm_f32, sizeof(float));

    // Send Left
    frame_left.id = this->get_parameter("motor_id_left").as_int();
    can_tx_pub_->publish(frame_left);

    // Send Right
    frame_right.id = this->get_parameter("motor_id_right").as_int();
    can_tx_pub_->publish(frame_right);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr limit_switch_pub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_pub_;
  
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
