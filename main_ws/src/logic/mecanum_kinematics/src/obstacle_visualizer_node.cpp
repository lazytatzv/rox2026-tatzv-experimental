// Copyright 2026 Tatsukiyano
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace mecanum_kinematics {

class ObstacleVisualizer : public rclcpp::Node {
public:
  ObstacleVisualizer() : Node("obstacle_visualizer") {
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacles", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ObstacleVisualizer::publish_markers, this));
    RCLCPP_INFO(this->get_logger(), "Obstacle Visualizer active");
  }

private:
  void publish_markers() {
    visualization_msgs::msg::MarkerArray array;

    // Stone 1 (Box)
    array.markers.push_back(make_marker(0, 1.0, 0.5, 0.05, 0.1, 0.1, 0.1, 0.5, 0.5, 0.5, visualization_msgs::msg::Marker::CUBE));
    // Stone 2 (Sphere)
    array.markers.push_back(make_marker(1, 1.5, -0.3, 0.05, 0.16, 0.16, 0.16, 0.4, 0.4, 0.4, visualization_msgs::msg::Marker::SPHERE));
    // Step 1 (Thin Plate)
    array.markers.push_back(make_marker(2, 2.5, 0.0, 0.01, 1.0, 2.0, 0.02, 0.7, 0.7, 0.7, visualization_msgs::msg::Marker::CUBE));
    // Stone 3 (Small Box)
    array.markers.push_back(make_marker(3, 0.8, -0.8, 0.05, 0.05, 0.1, 0.05, 0.3, 0.3, 0.3, visualization_msgs::msg::Marker::CUBE));

    publisher_->publish(array);
  }

  visualization_msgs::msg::Marker make_marker(int id, double x, double y, double z, double sx, double sy, double sz, double r, double g, double b, int type) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "odom";
    m.header.stamp = this->now();
    m.ns = "stones";
    m.id = id;
    m.type = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.scale.x = sx;
    m.scale.y = sy;
    m.scale.z = sz;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 0.8;
    return m;
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace mecanum_kinematics

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mecanum_kinematics::ObstacleVisualizer>());
  rclcpp::shutdown();
  return 0;
}
