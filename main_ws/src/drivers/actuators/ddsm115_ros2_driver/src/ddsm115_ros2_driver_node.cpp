#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_interfaces/msg/serial_frame.hpp"
#include "ddsm115_ros2_driver/ddsm115_ros2_driver_client.hpp"

using namespace std::chrono_literals;

namespace ddsm115_ros2_driver {

class DDSM115DriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit DDSM115DriverNode(const rclcpp::NodeOptions& options)
      : rclcpp_lifecycle::LifecycleNode("ddsm115_motor_node", options)
  {
    this->declare_parameter("motor_id", 1);
    this->declare_parameter("joint_name", "ddsm_joint");
    this->declare_parameter("topic_tx_queue", "/communication/tx_queue");
    this->declare_parameter("topic_rx_queue", "/communication/rx_queue");
    this->declare_parameter("topic_velocity_command", "~/velocity_command");
    this->declare_parameter("publish_rate", 20.0);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    motor_id_ = static_cast<uint8_t>(this->get_parameter("motor_id").as_int());
    joint_name_ = this->get_parameter("joint_name").as_string();
    topic_tx_queue_ = this->get_parameter("topic_tx_queue").as_string();
    topic_rx_queue_ = this->get_parameter("topic_rx_queue").as_string();
    topic_velocity_command_ = this->get_parameter("topic_velocity_command").as_string();

    driver_client_ = std::make_unique<DDSM115DriverClient>(
        std::bind(&DDSM115DriverNode::motor_feedback_callback, this, std::placeholders::_1),
        [this](LogLevel level, const std::string &msg) {
          switch (level) {
            case LogLevel::DEBUG: RCLCPP_DEBUG(get_logger(), "%s", msg.c_str()); break;
            case LogLevel::INFO:  RCLCPP_INFO(get_logger(), "%s", msg.c_str()); break;
            case LogLevel::WARN:  RCLCPP_WARN(get_logger(), "%s", msg.c_str()); break;
            case LogLevel::ERROR: RCLCPP_ERROR(get_logger(), "%s", msg.c_str()); break;
          }
        });

    auto sensor_qos = rclcpp::SensorDataQoS();
    auto command_qos = rclcpp::QoS(1).best_effort();

    publisher_serial_frames_ = this->create_publisher<robot_interfaces::msg::SerialFrame>(topic_tx_queue_, command_qos);
    publisher_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", sensor_qos);

    subscription_velocity_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        topic_velocity_command_, command_qos, std::bind(&DDSM115DriverNode::velocity_callback, this, std::placeholders::_1));

    subscription_serial_rx_ = this->create_subscription<robot_interfaces::msg::SerialFrame>(
        topic_rx_queue_, sensor_qos, std::bind(&DDSM115DriverNode::serial_rx_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Configured motor ID: %d", motor_id_);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    publisher_serial_frames_->on_activate();
    publisher_joint_state_->on_activate();
    auto packet = driver_client_->create_mode_command(motor_id_, ControlLoopModes::MODE_VELOCITY);
    publish_serial_frame(packet);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    auto packet = driver_client_->create_velocity_command(motor_id_, 0.0, true);
    publish_serial_frame(packet);
    publisher_serial_frames_->on_deactivate();
    publisher_joint_state_->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    publisher_serial_frames_.reset();
    publisher_joint_state_.reset();
    subscription_velocity_.reset();
    subscription_serial_rx_.reset();
    driver_client_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  void velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
    if (msg->data.empty()) return;
    double velocity_rad_s = msg->data[0];
    double velocity_rpm = velocity_rad_s * 60.0 / (2.0 * M_PI);
    auto packet = driver_client_->create_velocity_command(motor_id_, velocity_rpm);
    publish_serial_frame(packet);
  }

  void serial_rx_callback(const robot_interfaces::msg::SerialFrame::SharedPtr msg)
  {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
    driver_client_->feed_data(msg->frame_data);
  }

  void motor_feedback_callback(const std::vector<uint8_t> &packet)
  {
    if (packet.size() < 10 || packet[0] != motor_id_) return;
    int16_t velocity_rpm = (static_cast<int16_t>(packet[4]) << 8) | packet[5];
    double velocity_rad_s = static_cast<double>(velocity_rpm) * (2.0 * M_PI) / 60.0;
    auto joint_state = std::make_unique<sensor_msgs::msg::JointState>();
    joint_state->header.stamp = this->now();
    joint_state->name.push_back(joint_name_);
    joint_state->velocity.push_back(velocity_rad_s);
    publisher_joint_state_->publish(std::move(joint_state));
  }

  void publish_serial_frame(const std::vector<uint8_t> &data)
  {
    auto frame = std::make_unique<robot_interfaces::msg::SerialFrame>();
    frame->frame_data = data;
    publisher_serial_frames_->publish(std::move(frame));
  }

  uint8_t motor_id_;
  std::string joint_name_;
  std::string topic_tx_queue_;
  std::string topic_rx_queue_;
  std::string topic_velocity_command_;
  std::unique_ptr<DDSM115DriverClient> driver_client_;
  rclcpp_lifecycle::LifecyclePublisher<robot_interfaces::msg::SerialFrame>::SharedPtr publisher_serial_frames_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_state_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_velocity_;
  rclcpp::Subscription<robot_interfaces::msg::SerialFrame>::SharedPtr subscription_serial_rx_;
};

} // namespace ddsm115_ros2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ddsm115_ros2_driver::DDSM115DriverNode)
