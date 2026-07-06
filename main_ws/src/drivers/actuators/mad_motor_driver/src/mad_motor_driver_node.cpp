#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <thread>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

using namespace std::chrono_literals;

class MadMotorDriver : public rclcpp::Node
{
public:
  MadMotorDriver() : Node("mad_motor_driver_node"), current_target_rpm_(0.0), can_fd_(-1)
  {
    this->declare_parameter("can_interface", "can0");
    this->declare_parameter("motor_id_left", 0x201);
    this->declare_parameter("motor_id_right", 0x202);
    this->declare_parameter("limit_switch_id", 0x200);
    this->declare_parameter("watchdog_timeout", 0.5);

    watchdog_timeout_ = this->get_parameter("watchdog_timeout").as_double();
    std::string can_iface = this->get_parameter("can_interface").as_string();

    // 1. Initialize SocketCAN
    if (!init_socketcan(can_iface)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize SocketCAN on %s", can_iface.c_str());
    }

    // 2. Publisher for Limit Switch
    limit_switch_pub_ = this->create_publisher<std_msgs::msg::Bool>("/shooter/limit_switch", 10);

    // 3. Subscriber for Target RPM
    cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/shooter/cmd_muxed", 10,
      std::bind(&MadMotorDriver::cmd_callback, this, std::placeholders::_1));

    // 4. Watchdog Timer (stops motors if commands are lost)
    watchdog_timer_ = this->create_wall_timer(
      100ms, std::bind(&MadMotorDriver::watchdog_callback, this));

    // 5. CAN Tx Timer (sends Target RPM commands to STM32 at 100Hz)
    can_tx_timer_ = this->create_wall_timer(
      10ms, std::bind(&MadMotorDriver::can_tx_callback, this));

    last_cmd_time_ = this->now();

    // 6. Start CAN Rx Background Thread
    if (can_fd_ >= 0) {
      rx_thread_ = std::thread(&MadMotorDriver::can_rx_thread, this);
    }

    RCLCPP_INFO(this->get_logger(), "MAD Motor CAN Driver started on %s.", can_iface.c_str());
  }

  ~MadMotorDriver() {
    if (can_fd_ >= 0) {
      close(can_fd_);
    }
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
  }

private:
  bool init_socketcan(const std::string& iface_name) {
    can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd_ < 0) return false;

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, iface_name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) return false;

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) return false;
    return true;
  }

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
    if (can_fd_ < 0) return;

    // Send to Left Motor (0x201) and Right Motor (0x202)
    struct can_frame frame;
    frame.can_dlc = 4; // 4 bytes for float32

    // Convert double to float to match STM32 expectation
    float rpm_f32 = static_cast<float>(current_target_rpm_);
    std::memcpy(frame.data, &rpm_f32, sizeof(float));

    // Send Left
    frame.can_id = this->get_parameter("motor_id_left").as_int();
    write(can_fd_, &frame, sizeof(struct can_frame));

    // Send Right
    frame.can_id = this->get_parameter("motor_id_right").as_int();
    write(can_fd_, &frame, sizeof(struct can_frame));
  }

  void can_rx_thread()
  {
    struct can_frame frame;
    int limit_switch_id = this->get_parameter("limit_switch_id").as_int();

    while (rclcpp::ok()) {
      int nbytes = read(can_fd_, &frame, sizeof(struct can_frame));
      if (nbytes > 0) {
        if (frame.can_id == limit_switch_id && frame.can_dlc >= 1) {
          std_msgs::msg::Bool msg;
          msg.data = (frame.data[0] == 1);
          limit_switch_pub_->publish(msg);
        }
      }
    }
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr limit_switch_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr can_tx_timer_;
  
  std::thread rx_thread_;
  rclcpp::Time last_cmd_time_;
  int can_fd_;
  
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
