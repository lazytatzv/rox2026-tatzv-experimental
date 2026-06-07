// Copyright 2026 Tatsukiyano
#include <memory>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "gz_msgs/msg/scene.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace mecanum_kinematics {

/**
 * @brief Dynamic Obstacle Visualizer
 * Automatically converts Gazebo Scene information into ROS Markers.
 * No more manual hardcoding of stones!
 */
class DynamicObstacleVisualizer : public rclcpp::Node {
public:
  DynamicObstacleVisualizer() : Node("obstacle_visualizer") {
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacles", 10);
    subscription_ = this->create_subscription<gz_msgs::msg::Scene>(
      "/gz_scene", 10, std::bind(&DynamicObstacleVisualizer::scene_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Dynamic Obstacle Visualizer (Gazebo Ground Truth) active");
  }

private:
  void scene_callback(const gz_msgs::msg::Scene::SharedPtr msg) {
    visualization_msgs::msg::MarkerArray array;
    
    for (const auto & model : msg->models) {
      // Skip the robot itself to avoid visual clutter
      if (model.name == "lazytatzv_robot" || model.name == "ground_plane") continue;

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "odom";
      m.header.stamp = this->now();
      m.ns = "gazebo_world";
      m.id = model.id;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = model.pose; // Direct copy from Gazebo Truth
      
      // Default visualization for unknown shapes
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.2;

      // Special handling for the Big Wall
      if (model.name == "big_wall") {
        m.scale.x = 1.0; m.scale.y = 10.0; m.scale.z = 2.0;
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.8;
      } else {
        m.color.r = 0.5; m.color.g = 0.5; m.color.b = 0.5; m.color.a = 0.8;
      }

      array.markers.push_back(m);
    }
    publisher_->publish(array);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<gz_msgs::msg::Scene>::SharedPtr subscription_;
};

} // namespace mecanum_kinematics

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mecanum_kinematics::DynamicObstacleVisualizer>());
  rclcpp::shutdown();
  return 0;
}
