// Copyright 2026 Tatsukiyano
#include <memory>
#include "imu_stabilizer/stabilizer_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_stabilizer::HeadingStabilizerNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
