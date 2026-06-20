// Copyright 2026 Tatsukiyano
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class TwistToStampedNode : public rclcpp::Node
{
public:
  TwistToStampedNode() : Node("twist_to_stamped", rclcpp::NodeOptions().allow_undeclared_parameters(true))
  {
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_ext", 10);
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto stamped_msg = geometry_msgs::msg::TwistStamped();
        stamped_msg.header.stamp = this->now();
        stamped_msg.header.frame_id = "base_footprint";
        stamped_msg.twist = *msg;
        pub_->publish(stamped_msg);
      });
    RCLCPP_INFO(this->get_logger(), "TwistToStampedNode started. Relaying /cmd_vel -> /cmd_vel_ext");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToStampedNode>());
  rclcpp::shutdown();
  return 0;
}
