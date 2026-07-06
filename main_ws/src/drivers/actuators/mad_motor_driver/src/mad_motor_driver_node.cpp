#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <can_msgs/msg/frame.hpp>
#include <chrono>
#include <cstring>

using namespace std::chrono_literals;

class MadMotorDriver : public rclcpp::Node
{
public:
  MadMotorDriver() : Node("mad_motor_driver_node"), current_target_rpm_(0.0)
  {
    this->declare_parameter("cmd_id", 0x201);
    this->declare_parameter("imu_id", 0x202);
    this->declare_parameter("limit_switch_id", 0x200);
    this->declare_parameter("watchdog_timeout", 0.5);
    
    // 回転方向の反転設定 (向かい合わせ配置の場合は片方をマイナスにする必要があるため)
    this->declare_parameter("invert_top", false);
    this->declare_parameter("invert_bottom", true); // デフォルトで下側を逆回転と仮定
    
    // バックスピン比率
    this->declare_parameter("backspin_ratio", 1.0);
    
    watchdog_timeout_ = this->get_parameter("watchdog_timeout").as_double();

    // IMUのソース設定 ("stm32" または "rdk")
    this->declare_parameter("imu_source", "stm32");
    std::string imu_source = this->get_parameter("imu_source").as_string();

    // 1. Publisher for Limit Switch (Bool)
    limit_switch_pub_ = this->create_publisher<std_msgs::msg::Bool>("/shooter/limit_switch", 10);

    // 2. Publisher to ROS2 SocketCAN node
    can_tx_pub_ = this->create_publisher<can_msgs::msg::Frame>("/to_can", 10);

    // 3. Subscriber for Target RPM from Strategy/Teleop
    cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/shooter/cmd_muxed", 10,
      std::bind(&MadMotorDriver::cmd_callback, this, std::placeholders::_1));

    // 4. Subscriber from ROS2 SocketCAN node (Limit Switch & IMU RX)
    can_rx_sub_ = this->create_subscription<can_msgs::msg::Frame>(
      "/from_can", 10,
      std::bind(&MadMotorDriver::can_rx_callback, this, std::placeholders::_1));

    if (imu_source == "stm32") {
      // STM32がIMUを読み取り、RDKにCAN(0x202)で送ってくる場合
      imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
      RCLCPP_INFO(this->get_logger(), "IMU Source: STM32 -> RDK (Receiving 0x202)");
    } else {
      // RDKがIMUを読み取り、STM32にCAN(0x202)で送る場合
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        std::bind(&MadMotorDriver::imu_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "IMU Source: RDK -> STM32 (Sending 0x202)");
    }

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

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    can_msgs::msg::Frame frame_imu;
    frame_imu.is_rtr = false;
    frame_imu.is_extended = false;
    frame_imu.is_error = false;
    frame_imu.dlc = 8;
    frame_imu.id = this->get_parameter("imu_id").as_int();

    // Float -> Int16 (10000倍)
    int16_t w = static_cast<int16_t>(msg->orientation.w * 10000.0);
    int16_t x = static_cast<int16_t>(msg->orientation.x * 10000.0);
    int16_t y = static_cast<int16_t>(msg->orientation.y * 10000.0);
    int16_t z = static_cast<int16_t>(msg->orientation.z * 10000.0);

    std::memcpy(&frame_imu.data[0], &w, 2);
    std::memcpy(&frame_imu.data[2], &x, 2);
    std::memcpy(&frame_imu.data[4], &y, 2);
    std::memcpy(&frame_imu.data[6], &z, 2);

    can_tx_pub_->publish(frame_imu);
  }

  void can_rx_callback(const can_msgs::msg::Frame::SharedPtr msg)
  {
    uint32_t limit_switch_id = this->get_parameter("limit_switch_id").as_int();

    uint32_t imu_id = this->get_parameter("imu_id").as_int();

    if (msg->id == limit_switch_id && msg->dlc >= 1) {
      std_msgs::msg::Bool sw_msg;
      sw_msg.data = ((msg->data[0] & 0x01) != 0); // Bit 0 を取得
      limit_switch_pub_->publish(sw_msg);
    }
    else if (msg->id == imu_id && msg->dlc >= 8 && imu_pub_) {
      // STM32から送られてきたIMUデータを復元 (RDK側で使用する場合)
      sensor_msgs::msg::Imu imu_msg;
      
      int16_t w, x, y, z;
      std::memcpy(&w, &msg->data[0], 2);
      std::memcpy(&x, &msg->data[2], 2);
      std::memcpy(&y, &msg->data[4], 2);
      std::memcpy(&z, &msg->data[6], 2);

      imu_msg.orientation.w = static_cast<double>(w) / 10000.0;
      imu_msg.orientation.x = static_cast<double>(x) / 10000.0;
      imu_msg.orientation.y = static_cast<double>(y) / 10000.0;
      imu_msg.orientation.z = static_cast<double>(z) / 10000.0;
      
      imu_pub_->publish(imu_msg);
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
    can_msgs::msg::Frame frame_cmd;

    frame_cmd.is_rtr = false;
    frame_cmd.is_extended = false;
    frame_cmd.is_error = false;
    frame_cmd.dlc = 8;
    frame_cmd.id = this->get_parameter("cmd_id").as_int();

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

    // Int16にキャスト
    int16_t rpm_top_i16 = static_cast<int16_t>(target_rpm_top);
    int16_t rpm_bottom_i16 = static_cast<int16_t>(target_rpm_bottom);
    int16_t rpm_dribbler_i16 = 0; // ドリブラーは未実装のため0
    uint8_t estop = 0;            // 非常停止フラグ (未実装のため0)
    uint8_t reserved = 0;         // 予備

    // データのパッキング (リトルエンディアン)
    std::memcpy(&frame_cmd.data[0], &rpm_top_i16, 2);
    std::memcpy(&frame_cmd.data[2], &rpm_bottom_i16, 2);
    std::memcpy(&frame_cmd.data[4], &rpm_dribbler_i16, 2);
    frame_cmd.data[6] = estop;
    frame_cmd.data[7] = reserved;

    can_tx_pub_->publish(frame_cmd);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr limit_switch_pub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  
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
