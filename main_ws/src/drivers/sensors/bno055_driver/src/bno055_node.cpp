// Copyright 2026 Tatsukiyano
// The Ultimate BNO055 ROS 2 Driver - Auto-Recovery, Ext Crystal, Calibration, Zero-Copy Component

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace bno055_driver {

class BNO055Node : public rclcpp::Node {
public:
  explicit BNO055Node(const rclcpp::NodeOptions & options)
  : Node("bno055_node", options), i2c_fd_(-1), is_initialized_(false)
  {
    declare_parameters();
    
    // Set up publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS());
    temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/imu/temperature", 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    // Try initialization in a background thread to not block launch
    init_thread_ = std::make_unique<std::thread>(&BNO055Node::initialize_bno055, this);

    // Timers
    timer_ = this->create_wall_timer(10ms, std::bind(&BNO055Node::read_and_publish, this));
    diag_timer_ = this->create_wall_timer(1s, std::bind(&BNO055Node::publish_diagnostics, this));

    RCLCPP_INFO(get_logger(), "BNO055 Ultimate Driver Node Started (Initializing...)");
  }

  ~BNO055Node() override {
    if (init_thread_ && init_thread_->joinable()) {
      init_thread_->join();
    }
    if (i2c_fd_ >= 0) {
      close(i2c_fd_);
    }
  }

private:
  void declare_parameters()
  {
    this->declare_parameter("i2c_bus", "/dev/i2c-1");
    this->declare_parameter("i2c_addr", 0x28);
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("variance_orientation", 0.001);
    this->declare_parameter("variance_angular_velocity", 0.01);
    this->declare_parameter("variance_linear_acceleration", 0.1);
    
    // Axis Remapping
    this->declare_parameter("axis_map_config", 0x24); // Default P1
    this->declare_parameter("axis_map_sign", 0x00);
    
    // Ext Crystal
    this->declare_parameter("use_ext_crystal", true);

    // Calibration Offsets (Enable setting them on boot!)
    this->declare_parameter("load_calibration", false);
    // 22 bytes of calibration data
    this->declare_parameter("calibration_data", std::vector<int64_t>(22, 0)); 
  }

  bool write8(uint8_t reg, uint8_t val)
  {
    if (i2c_fd_ < 0) return false;
    uint8_t buf[2] = {reg, val};
    if (write(i2c_fd_, buf, 2) != 2) {
      return false;
    }
    std::this_thread::sleep_for(2ms);
    return true;
  }

  uint8_t read8(uint8_t reg)
  {
    if (i2c_fd_ < 0) return 0;
    uint8_t buf[1] = {reg};
    if (write(i2c_fd_, buf, 1) != 1) return 0;
    if (read(i2c_fd_, buf, 1) != 1) return 0;
    return buf[0];
  }

  void initialize_bno055()
  {
    std::string bus = this->get_parameter("i2c_bus").as_string();
    int addr = this->get_parameter("i2c_addr").as_int();

    while (rclcpp::ok() && !is_initialized_) {
      if (i2c_fd_ < 0) {
        i2c_fd_ = open(bus.c_str(), O_RDWR);
        if (i2c_fd_ >= 0 && ioctl(i2c_fd_, I2C_SLAVE, addr) >= 0) {
          RCLCPP_INFO(get_logger(), "I2C connected to %s at 0x%02x", bus.c_str(), addr);
        } else {
          if (i2c_fd_ >= 0) { close(i2c_fd_); i2c_fd_ = -1; }
          std::this_thread::sleep_for(1s);
          continue;
        }
      }

      // 1. Check ID
      uint8_t id = read8(0x00);
      if (id != 0xA0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for BNO055 (Got ID 0x%02x, expected 0xA0)...", id);
        std::this_thread::sleep_for(1s);
        continue;
      }

      // 2. CONFIG Mode
      write8(0x3D, 0x00);
      std::this_thread::sleep_for(30ms);

      // 3. Reset
      write8(0x3F, 0x20);
      std::this_thread::sleep_for(30ms);
      while (rclcpp::ok() && read8(0x00) != 0xA0) {
        std::this_thread::sleep_for(10ms);
      }
      std::this_thread::sleep_for(50ms);

      // 4. Normal Power
      write8(0x3E, 0x00);
      std::this_thread::sleep_for(10ms);
      write8(0x07, 0x00); // PAGE_ID = 0

      // 5. Axis Remap
      uint8_t map_config = this->get_parameter("axis_map_config").as_int();
      uint8_t map_sign = this->get_parameter("axis_map_sign").as_int();
      write8(0x41, map_config);
      write8(0x42, map_sign);

      // 6. Set Calibration Offsets if requested
      if (this->get_parameter("load_calibration").as_bool()) {
        auto calib = this->get_parameter("calibration_data").as_integer_array();
        if (calib.size() == 22) {
          for (size_t i = 0; i < 22; i++) {
            write8(0x55 + i, calib[i]); // ACCEL_OFFSET_X_LSB_ADDR starts at 0x55
          }
          RCLCPP_INFO(get_logger(), "Injected 22 bytes of calibration data into BNO055.");
        }
      }

      // 7. External Crystal (Super Stable)
      if (this->get_parameter("use_ext_crystal").as_bool()) {
        write8(0x3F, 0x80);
        std::this_thread::sleep_for(10ms);
      }

      // 8. NDOF Mode (Ultimate Sensor Fusion)
      write8(0x3D, 0x0C);
      std::this_thread::sleep_for(50ms);

      is_initialized_ = true;
      RCLCPP_INFO(get_logger(), "BNO055 Initialization Complete! Running in NDOF mode.");
    }
  }

  void read_and_publish()
  {
    if (!is_initialized_ || i2c_fd_ < 0) return;

    // Burst read 26 bytes starting from GYRO_DATA_X_LSB (0x14)
    // 0x14 - 0x19: Gyro (6)
    // 0x1A - 0x1F: Euler (6)
    // 0x20 - 0x27: Quat (8)
    // 0x28 - 0x2D: Linear Accel (6)
    uint8_t start_reg = 0x14;
    uint8_t data[26];

    if (write(i2c_fd_, &start_reg, 1) != 1) {
      error_count_++;
      check_i2c_health();
      return;
    }
    if (read(i2c_fd_, data, 26) != 26) {
      error_count_++;
      check_i2c_health();
      return;
    }
    error_count_ = 0; // Success resets error counter

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = this->get_parameter("frame_id").as_string();

    auto scale_quat = 1.0 / (1 << 14);
    msg.orientation.w = ((int16_t)(data[13] << 8 | data[12])) * scale_quat; // 0x20 (index 12)
    msg.orientation.x = ((int16_t)(data[15] << 8 | data[14])) * scale_quat;
    msg.orientation.y = ((int16_t)(data[17] << 8 | data[16])) * scale_quat;
    msg.orientation.z = ((int16_t)(data[19] << 8 | data[18])) * scale_quat;

    // 0x28 (index 20) is Linear Accel
    msg.linear_acceleration.x = ((int16_t)(data[21] << 8 | data[20])) / 100.0;
    msg.linear_acceleration.y = ((int16_t)(data[23] << 8 | data[22])) / 100.0;
    msg.linear_acceleration.z = ((int16_t)(data[25] << 8 | data[24])) / 100.0;

    // 0x14 (index 0) is Gyro
    msg.angular_velocity.x = ((int16_t)(data[1] << 8 | data[0])) / 900.0;
    msg.angular_velocity.y = ((int16_t)(data[3] << 8 | data[2])) / 900.0;
    msg.angular_velocity.z = ((int16_t)(data[5] << 8 | data[4])) / 900.0;

    double var_o = this->get_parameter("variance_orientation").as_double();
    double var_a = this->get_parameter("variance_angular_velocity").as_double();
    double var_l = this->get_parameter("variance_linear_acceleration").as_double();

    for(int i = 0; i < 9; i += 4) {
      msg.orientation_covariance[i] = var_o;
      msg.angular_velocity_covariance[i] = var_a;
      msg.linear_acceleration_covariance[i] = var_l;
    }

    imu_pub_->publish(msg);
  }

  void publish_diagnostics()
  {
    if (!is_initialized_) return;

    // Read Calibration Status (0x35)
    uint8_t calib = read8(0x35);
    uint8_t sys = (calib >> 6) & 0x03;
    uint8_t gyro = (calib >> 4) & 0x03;
    uint8_t accel = (calib >> 2) & 0x03;
    uint8_t mag = calib & 0x03;

    auto diag_msg = diagnostic_msgs::msg::DiagnosticArray();
    diag_msg.header.stamp = this->get_clock()->now();
    
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "BNO055 IMU Status";
    status.hardware_id = this->get_parameter("i2c_bus").as_string();
    
    if (sys == 3 && gyro == 3 && accel == 3 && mag == 3) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Fully Calibrated";
    } else if (gyro == 3) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Partially Calibrated (Gyro OK)";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Uncalibrated - Please move the robot in 8-figure shape";
    }

    auto add_kv = [&status](const std::string& k, const std::string& v) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = k;
      kv.value = v;
      status.values.push_back(kv);
    };

    add_kv("Sys_Calib", std::to_string(sys) + "/3");
    add_kv("Gyro_Calib", std::to_string(gyro) + "/3");
    add_kv("Accel_Calib", std::to_string(accel) + "/3");
    add_kv("Mag_Calib", std::to_string(mag) + "/3");

    // Read Temp
    int8_t temp = read8(0x34);
    add_kv("Temperature_C", std::to_string(temp));

    diag_msg.status.push_back(status);
    diag_pub_->publish(diag_msg);

    auto temp_msg = sensor_msgs::msg::Temperature();
    temp_msg.header.stamp = diag_msg.header.stamp;
    temp_msg.header.frame_id = this->get_parameter("frame_id").as_string();
    temp_msg.temperature = temp;
    temp_pub_->publish(temp_msg);
  }

  void check_i2c_health() {
    if (error_count_ > 10) {
      RCLCPP_ERROR(get_logger(), "I2C Bus Error Threshold Reached! Triggering Hard Recovery...");
      is_initialized_ = false;
      close(i2c_fd_);
      i2c_fd_ = -1;
      error_count_ = 0;
      
      if (init_thread_ && init_thread_->joinable()) {
        init_thread_->join();
      }
      init_thread_ = std::make_unique<std::thread>(&BNO055Node::initialize_bno055, this);
    }
  }

  int i2c_fd_;
  std::atomic<bool> is_initialized_;
  int error_count_ = 0;
  std::unique_ptr<std::thread> init_thread_;
  
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

} // namespace bno055_driver

RCLCPP_COMPONENTS_REGISTER_NODE(bno055_driver::BNO055Node)
