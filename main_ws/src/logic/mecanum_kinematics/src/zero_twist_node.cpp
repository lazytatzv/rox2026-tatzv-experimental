// Copyright 2026 Tatsukiyano
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

namespace mecanum_kinematics {

class ZeroTwistNode : public rclcpp::Node {
public:
  ZeroTwistNode() : Node("zero_twist_node") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_idle", 10);
    timer_ = this->create_wall_timer(50ms, [this]() {
      auto msg = std::make_unique<geometry_msgs::msg::Twist>();
      publisher_->publish(std::move(msg));
    });
    RCLCPP_INFO(this->get_logger(), "Zero Twist Baseline active at 20Hz");
  }
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace mecanum_kinematics

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mecanum_kinematics::ZeroTwistNode>());
  rclcpp::shutdown();
  return 0;
}
