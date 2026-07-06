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
    this->declare_parameter("motor_id_top", 0x201);
    this->declare_parameter("motor_id_bottom", 0x202);
    this->declare_parameter("limit_switch_id", 0x200);
    this->declare_parameter("watchdog_timeout", 0.5);
    
    // 回転方向の反転設定 (向かい合わせ配置の場合は片方をマイナスにする必要があるため)
    this->declare_parameter("invert_top", false);
    this->declare_parameter("invert_bottom", true); // デフォルトで下側を逆回転と仮定
    
    // バックスピン比率 (1.0で上下同じ速度。1.2なら下側が20%速く回り、バックスピンがかかる)
    this->declare_parameter("backspin_ratio", 1.0);

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

    RCLCPP_INFO(this->get_logger(), "MAD Motor CAN Driver started (Top/Bottom Mode).");
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
    can_msgs::msg::Frame frame_top;
    can_msgs::msg::Frame frame_bottom;

    frame_top.is_rtr = false;
    frame_top.is_extended = false;
    frame_top.is_error = false;
    frame_top.dlc = 4;

    frame_bottom.is_rtr = false;
    frame_bottom.is_extended = false;
    frame_bottom.is_error = false;
    frame_bottom.dlc = 4;

    // パラメータ取得
    bool invert_top = this->get_parameter("invert_top").as_bool();
    bool invert_bottom = this->get_parameter("invert_bottom").as_bool();
    double backspin_ratio = this->get_parameter("backspin_ratio").as_double();

    // RPMの計算 (下側をバックスピン比率に合わせて速くする)
    double target_rpm_top = current_target_rpm_;
    double target_rpm_bottom = current_target_rpm_ * backspin_ratio;

    // 反転設定の適用
    if (invert_top) target_rpm_top = -target_rpm_top;
    if (invert_bottom) target_rpm_bottom = -target_rpm_bottom;

    float rpm_top_f32 = static_cast<float>(target_rpm_top);
    float rpm_bottom_f32 = static_cast<float>(target_rpm_bottom);

    std::memcpy(frame_top.data.data(), &rpm_top_f32, sizeof(float));
    std::memcpy(frame_bottom.data.data(), &rpm_bottom_f32, sizeof(float));

    // Send Top
    frame_top.id = this->get_parameter("motor_id_top").as_int();
    can_tx_pub_->publish(frame_top);

    // Send Bottom
    frame_bottom.id = this->get_parameter("motor_id_bottom").as_int();
    can_tx_pub_->publish(frame_bottom);
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
