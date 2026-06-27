// Copyright 2026 Tatsukiyano
#include "robstride_driver/robstride_system_hardware.hpp"

#include <cmath>
#include <algorithm>
#include <string>
#include <vector>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robstride_driver/at_protocol_handler.hpp"
#include "robstride_driver/ddsm_protocol_handler.hpp"
#include "robstride_driver/private_protocol_handler.hpp"

namespace robstride_driver
{

hardware_interface::CallbackReturn RobstrideSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  node_ = std::make_shared<rclcpp::Node>("robstride_hardware_interface_node");

  // Determine Protocol and Transport Layer Type
  protocol_type_ = "at_gateway";
  if (info_.hardware_parameters.count("protocol")) {
    protocol_type_ = info_.hardware_parameters.at("protocol");
  }

  transport_type_ = "serial_port";
  if (info_.hardware_parameters.count("transport")) {
    transport_type_ = info_.hardware_parameters.at("transport");
  }

  // 1. Protocol Validation & Initialization
  if (protocol_type_ == "at_gateway") {
    double max_speed_percentage = 50.0;
    if (info_.hardware_parameters.count("max_speed_limit_percentage")) {
      max_speed_percentage = std::stod(info_.hardware_parameters.at("max_speed_limit_percentage"));
    }
    int max_delta = static_cast<int>(at_protocol::NEUTRAL_VELOCITY_VALUE *
      (max_speed_percentage / 100.0));
    protocol_handler_ = std::make_unique<AtProtocolHandler>(vel_max_, max_delta);
  } else if (protocol_type_ == "native_can") {
    float kp = 6.0f;
    float ki = 0.02f;
    float limit_cur = 5.0f;
    if (info_.hardware_parameters.count("kp")) kp = std::stof(info_.hardware_parameters.at("kp"));
    if (info_.hardware_parameters.count("ki")) ki = std::stof(info_.hardware_parameters.at("ki"));
    if (info_.hardware_parameters.count("limit_cur")) limit_cur = std::stof(info_.hardware_parameters.at("limit_cur"));
    protocol_handler_ = std::make_unique<PrivateProtocolHandler>(0x00, kp, ki, limit_cur);
  } else if (protocol_type_ == "ddsm") {
    protocol_handler_ = std::make_unique<DdsmProtocolHandler>();
  } else {
    RCLCPP_FATAL(
      node_->get_logger(),
      "Unknown protocol: %s (Must be 'at_gateway', 'native_can', or 'ddsm')",
      protocol_type_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 2. Transport Validation & Initialization
  if (transport_type_ == "serial_port") {
    // Initialize Transport
    transport_ = std::make_unique<RobstrideSerialDriver>();
    transport_->set_receive_callback(std::bind(&RobstrideSystemHardware::can_rx_callback, this,
        std::placeholders::_1));
  } else if (transport_type_ == "socketcan") {
    // Direct Linux SocketCAN for lowest latency
  } else if (transport_type_ == "ros_topic") {
    // Legacy SocketCAN mode via ros2_socketcan bridge using ROS topic
  } else {
    RCLCPP_FATAL(
      node_->get_logger(),
      "Unknown transport: %s (Must be 'serial_port', 'socketcan', or 'ros_topic')",
      transport_type_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize motors from URDF joints
  motors_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    motors_[i].id = std::stoi(joint.parameters.at("motor_id"), nullptr, 0);
    const std::string & invert_str = joint.parameters.at("invert_direction");
    motors_[i].invert = (invert_str == "true" || invert_str == "True" || invert_str == "1");
    id_to_index_[motors_[i].id] = i;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (transport_type_ == "serial_port") {
    RobstrideSerialConfig config;
    if (info_.hardware_parameters.count("usb_path")) {
      config.usb_path = info_.hardware_parameters.at("usb_path");
    }
    if (info_.hardware_parameters.count("serial_baud")) {
      config.serial_baud = std::stoi(info_.hardware_parameters.at("serial_baud"));
    }

    try {
      transport_->open(config);
      RCLCPP_INFO(node_->get_logger(), "Transport opened on %s", config.usb_path.c_str());
    } catch (const std::exception & e) {
      RCLCPP_FATAL(node_->get_logger(), "Failed to open transport: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (transport_type_ == "socketcan") {
    if (info_.hardware_parameters.count("can_interface")) {
      can_interface_ = info_.hardware_parameters.at("can_interface");
    }

    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_FATAL(node_->get_logger(), "Error creating socket for SocketCAN");
      return hardware_interface::CallbackReturn::ERROR;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_FATAL(node_->get_logger(), "Error finding CAN interface: %s", can_interface_.c_str());
      close(can_socket_);
      return hardware_interface::CallbackReturn::ERROR;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Set receive timeout to allow thread to exit smoothly
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms
    setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_FATAL(node_->get_logger(), "Error binding SocketCAN to %s", can_interface_.c_str());
      close(can_socket_);
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(node_->get_logger(), "Direct SocketCAN initialized on %s (Ultimate Performance Mode)", can_interface_.c_str());
  } else if (transport_type_ == "ros_topic") {
    std::string rx_topic = "/from_can_bus";
    std::string tx_topic = "/to_can_bus";
    if (info_.hardware_parameters.count("can_rx_topic")) {
      rx_topic = info_.hardware_parameters.at("can_rx_topic");
    }
    if (info_.hardware_parameters.count("can_tx_topic")) {
      tx_topic = info_.hardware_parameters.at("can_tx_topic");
    }

    can_pub_ = node_->create_publisher<can_msgs::msg::Frame>(tx_topic, 100);
    can_sub_ = node_->create_subscription<can_msgs::msg::Frame>(
      rx_topic, 100,
      std::bind(&RobstrideSystemHardware::can_rx_topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "ROS topic communication initialized: pub=%s, sub=%s",
        tx_topic.c_str(), rx_topic.c_str());
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (transport_type_ == "serial_port") {
    transport_->close();
  } else if (transport_type_ == "socketcan") {
    if (can_socket_ >= 0) {
      close(can_socket_);
      can_socket_ = -1;
    }
  } else if (transport_type_ == "ros_topic") {
    can_pub_.reset();
    can_sub_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (transport_type_ == "socketcan") {
    can_rx_running_ = true;
    can_rx_thread_ = std::make_unique<std::thread>(&RobstrideSystemHardware::can_rx_thread_func, this);

    // Set thread priority to real-time for zero-latency CAN RX
    struct sched_param param;
    param.sched_priority = 80;
    if (pthread_setschedparam(can_rx_thread_->native_handle(), SCHED_FIFO, &param) != 0) {
      RCLCPP_WARN(node_->get_logger(), "Failed to set SCHED_FIFO for CAN RX thread. Requires sudo/root for ultimate latency.");
    }
  } else if (transport_type_ == "ros_topic") {
    executor_.add_node(node_);
    spin_thread_ = std::make_unique<std::thread>([this]() {
          executor_.spin();
    });
  }

  // 1. Send Mode Select Command (to Velocity mode) for ALL motors
  for (const auto & motor : motors_) {
    auto mode_data = protocol_handler_->create_mode_select_command(motor.id, "velocity");
    if (!mode_data.empty()) {
      if (transport_type_ == "serial_port") {
        for (int i = 0; i < 3; ++i) {
          send_command(motor.id, mode_data);
          std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Python: time.sleep(0.02)
        }
      } else {
        send_command(motor.id, mode_data);
      }
    }
  }
  if (transport_type_ == "serial_port") {
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Python: time.sleep(0.5)
  }

  // 2. Send Enable Command for ALL motors
  for (const auto & motor : motors_) {
    auto frame_data = protocol_handler_->create_enable_command(motor.id);
    if (!frame_data.empty()) {
      if (transport_type_ == "serial_port") {
        for (int i = 0; i < 3; ++i) {
          send_command(motor.id, frame_data);
          std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Python: time.sleep(0.02)
        }
      } else {
        send_command(motor.id, frame_data);
      }
    }
  }
  if (transport_type_ == "serial_port") {
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Python: time.sleep(0.5)
  }

  // 3. Send Initial Command (0.0 rad/s) for safety for ALL motors
  for (const auto & motor : motors_) {
    auto init_data = protocol_handler_->create_velocity_command(motor.id, 0.0);
    if (!init_data.empty()) {
      send_command(motor.id, init_data);
      if (transport_type_ == "serial_port") {
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Python: time.sleep(0.02)
      }
    }
  }
  if (transport_type_ == "serial_port") {
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Python: time.sleep(0.1)
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobstrideSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (const auto & motor : motors_) {
    auto frame_data = protocol_handler_->create_disable_command(motor.id);
    if (!frame_data.empty()) {
      if (transport_type_ == "serial_port") {
        // Send disable 3 times per motor with delay, matching motor_exact_run.py
        for (int i = 0; i < 3; ++i) {
          send_command(motor.id, frame_data);
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      } else {
        send_command(motor.id, frame_data);
      }
    }
  }

  if (transport_type_ == "socketcan") {
    can_rx_running_ = false;
    if (can_rx_thread_ && can_rx_thread_->joinable()) {
      can_rx_thread_->join();
    }
    can_rx_thread_.reset();
  } else if (transport_type_ == "ros_topic") {
    executor_.cancel();
    if (spin_thread_ && spin_thread_->joinable()) {
      spin_thread_->join();
    }
    spin_thread_.reset();
    executor_.remove_node(node_);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobstrideSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motors_[i].position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors_[i].velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &motors_[i].effort));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobstrideSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors_[i].command_velocity));
  }
  return command_interfaces;
}

hardware_interface::return_type RobstrideSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Non-blocking read is handled by can_rx_callback and mutex
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobstrideSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & motor : motors_) {
    double velocity = motor.command_velocity;
    if (motor.invert) {velocity = -velocity;}

    auto frame_data = protocol_handler_->create_velocity_command(motor.id, velocity);
    if (!frame_data.empty()) {
      send_command(motor.id, frame_data);
      if (transport_type_ == "serial_port") {
        // 0.5ms inter-frame delay: gives the serial-to-CAN board time to
        // process each frame without the ~3ms overhead of tcdrain().
        // 4 motors × 0.5ms = 2ms total, within the 10ms cycle budget.
        usleep(500);
      }
    }
  }
  return hardware_interface::return_type::OK;
}

void RobstrideSystemHardware::can_rx_callback(const std::vector<uint8_t> & frame)
{
  process_received_frame(frame);
}

void RobstrideSystemHardware::can_rx_topic_callback(const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  std::vector<uint8_t> frame_data;
  if (protocol_type_ == "native_can") {
    frame_data.resize(16, 0);
    frame_data[0] = 0xAA;
    std::memcpy(&frame_data[1], &msg->id, 4);
    frame_data[5] = msg->is_extended ? 0x01 : 0x00;
    frame_data[6] = msg->is_rtr ? 0x01 : 0x00;
    frame_data[7] = msg->dlc;
    std::memcpy(&frame_data[8], msg->data.data(), 8);
  } else if (protocol_type_ == "ddsm") {
    frame_data.resize(10, 0);
    frame_data[0] = msg->id & 0xFF;
    std::memcpy(&frame_data[1], msg->data.data(), 8);
  } else {
    // AT protocol fallback
    frame_data.assign(msg->data.begin(), msg->data.begin() + msg->dlc);
  }

  if (!frame_data.empty()) {
    process_received_frame(frame_data);
  }
}

void RobstrideSystemHardware::send_command(uint8_t motor_id, const std::vector<uint8_t> & frame_data)
{
  if (frame_data.empty()) {return;}

  if (transport_type_ == "serial_port") {
    transport_->send_raw(frame_data);
  } else if (transport_type_ == "socketcan") {
    if (can_socket_ < 0) return;
    
    if (protocol_type_ == "native_can") {
      for (size_t offset = 0; offset + 16 <= frame_data.size(); offset += 16) {
        struct can_frame frame;
        std::memset(&frame, 0, sizeof(struct can_frame));
        uint32_t can_id;
        std::memcpy(&can_id, &frame_data[offset + 1], 4);
        
        bool is_ext = (frame_data[offset + 5] == 0x01);
        bool is_rtr = (frame_data[offset + 6] == 0x01);
        frame.can_id = can_id;
        if (is_ext) frame.can_id |= CAN_EFF_FLAG;
        if (is_rtr) frame.can_id |= CAN_RTR_FLAG;
        
        frame.can_dlc = frame_data[offset + 7];
        std::memcpy(frame.data, &frame_data[offset + 8], 8);
        
        ::write(can_socket_, &frame, sizeof(struct can_frame));
      }
    } else if (protocol_type_ == "ddsm") {
      struct can_frame frame;
      std::memset(&frame, 0, sizeof(struct can_frame));
      frame.can_id = frame_data[0];
      frame.can_dlc = 8;
      std::memcpy(frame.data, &frame_data[1], 8);
      ::write(can_socket_, &frame, sizeof(struct can_frame));
    } else {
      struct can_frame frame;
      std::memset(&frame, 0, sizeof(struct can_frame));
      frame.can_id = motor_id;
      frame.can_dlc = std::min(static_cast<size_t>(frame_data.size()), static_cast<size_t>(8));
      std::memcpy(frame.data, frame_data.data(), frame.can_dlc);
      ::write(can_socket_, &frame, sizeof(struct can_frame));
    }
  } else if (transport_type_ == "ros_topic") {
    if (protocol_type_ == "native_can") {
      for (size_t offset = 0; offset + 16 <= frame_data.size(); offset += 16) {
        can_msgs::msg::Frame msg;
        msg.header.stamp = node_->get_clock()->now();
        uint32_t can_id;
        std::memcpy(&can_id, &frame_data[offset + 1], 4);
        msg.id = can_id;
        msg.is_extended = (frame_data[offset + 5] == 0x01);
        msg.is_rtr = (frame_data[offset + 6] == 0x01);
        msg.dlc = frame_data[offset + 7];
        std::memcpy(msg.data.data(), &frame_data[offset + 8], 8);
        can_pub_->publish(msg);
      }
    } else if (protocol_type_ == "ddsm") {
      can_msgs::msg::Frame msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.id = frame_data[0];
      msg.is_extended = false;
      msg.is_rtr = false;
      msg.dlc = 8;
      std::memcpy(msg.data.data(), &frame_data[1], 8);
      can_pub_->publish(msg);
    } else {
      // Fallback/AT mode
      can_msgs::msg::Frame msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.id = motor_id;
      msg.is_extended = false;
      msg.is_rtr = false;
      msg.dlc = std::min(static_cast<size_t>(frame_data.size()), static_cast<size_t>(8));
      std::memcpy(msg.data.data(), frame_data.data(), msg.dlc);
      can_pub_->publish(msg);
    }
  }
}

void RobstrideSystemHardware::process_received_frame(const std::vector<uint8_t> & frame_data)
{
  auto result = protocol_handler_->decode_frame(frame_data);
  if (!result.success) {return;}

  uint8_t motor_id = result.motor_id;
  if (id_to_index_.find(motor_id) == id_to_index_.end()) {return;}

  size_t idx = id_to_index_[motor_id];
  auto & state = result.state;

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (motors_[idx].invert) {
    motors_[idx].position = -state.position;
    motors_[idx].velocity = -state.velocity;
    motors_[idx].effort = -state.effort;
  } else {
    motors_[idx].position = state.position;
    motors_[idx].velocity = state.velocity;
    motors_[idx].effort = state.effort;
  }
}

void RobstrideSystemHardware::can_rx_thread_func()
{
  while (can_rx_running_) {
    struct can_frame frame;
    int nbytes = ::read(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes > 0 && (size_t)nbytes == sizeof(struct can_frame)) {
      std::vector<uint8_t> frame_data;
      if (protocol_type_ == "native_can") {
        frame_data.resize(16, 0);
        frame_data[0] = 0xAA;
        uint32_t can_id = frame.can_id & CAN_EFF_MASK;
        std::memcpy(&frame_data[1], &can_id, 4);
        frame_data[5] = (frame.can_id & CAN_EFF_FLAG) ? 0x01 : 0x00;
        frame_data[6] = (frame.can_id & CAN_RTR_FLAG) ? 0x01 : 0x00;
        frame_data[7] = frame.can_dlc;
        std::memcpy(&frame_data[8], frame.data, 8);
      } else if (protocol_type_ == "ddsm") {
        frame_data.resize(10, 0);
        frame_data[0] = frame.can_id & 0xFF;
        std::memcpy(&frame_data[1], frame.data, 8);
      } else {
        frame_data.assign(frame.data, frame.data + frame.can_dlc);
      }
      process_received_frame(frame_data);
    }
  }
}

}  // namespace robstride_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robstride_driver::RobstrideSystemHardware, hardware_interface::SystemInterface)
