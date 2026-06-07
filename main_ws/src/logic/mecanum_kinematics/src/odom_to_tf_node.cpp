// Copyright 2026 Tatsukiyano
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace mecanum_kinematics {

class OdomToTFNode : public rclcpp::Node {
public:
  OdomToTFNode() : Node("odom_to_tf_node") {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&OdomToTFNode::odom_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Odom-to-TF Bridge active");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";
    
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

} // namespace mecanum_kinematics

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mecanum_kinematics::OdomToTFNode>());
  rclcpp::shutdown();
  return 0;
}
