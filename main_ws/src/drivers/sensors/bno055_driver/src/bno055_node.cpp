// Copyright 2026 Tatsukiyano
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class BNO055Node : public rclcpp::Node {
 public:
  BNO055Node() : Node("bno055_node") {
    declare_parameters();
    setup_i2c();
    init_bno055();

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&BNO055Node::read_and_publish, this));

    RCLCPP_INFO(get_logger(), "BNO055 Pro Driver Ready (Warning-Free)");
  }

 private:
  void declare_parameters() {
    this->declare_parameter("i2c_bus", "/dev/i2c-1");
    this->declare_parameter("i2c_addr", 0x28);
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("variance_orientation", 0.001);
    this->declare_parameter("variance_angular_velocity", 0.01);
    this->declare_parameter("variance_linear_acceleration", 0.1);
  }

  void setup_i2c() {
    std::string bus = this->get_parameter("i2c_bus").as_string();
    int addr = this->get_parameter("i2c_addr").as_int();
    i2c_fd_ = open(bus.c_str(), O_RDWR);
    if (i2c_fd_ < 0 || ioctl(i2c_fd_, I2C_SLAVE, addr) < 0) {
      RCLCPP_FATAL(get_logger(), "I2C Setup Failed on %s at 0x%02x", bus.c_str(), addr);
    }
  }

  bool write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    if (write(i2c_fd_, buf, 2) != 2) {
        RCLCPP_ERROR(get_logger(), "I2C Write Failed to reg 0x%02x", reg);
        return false;
    }
    usleep(2000);
    return true;
  }

  void init_bno055() {
    write_reg(0x3D, 0x00); // Config
    write_reg(0x3E, 0x00); // Power
    write_reg(0x3D, 0x0C); // NDOF Mode
    usleep(20000);
  }

  void read_and_publish() {
    uint8_t start_reg = 0x14;
    uint8_t data[26]; 
    
    // Set address and check result to suppress warnings
    if (write(i2c_fd_, &start_reg, 1) != 1) return;
    
    if (read(i2c_fd_, data, 26) != 26) return;

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = this->get_parameter("frame_id").as_string();

    auto scale_quat = 1.0 / (1 << 14);
    msg.orientation.w = ((int16_t)(data[7] << 8 | data[6])) * scale_quat;
    msg.orientation.x = ((int16_t)(data[9] << 8 | data[8])) * scale_quat;
    msg.orientation.y = ((int16_t)(data[11] << 8 | data[10])) * scale_quat;
    msg.orientation.z = ((int16_t)(data[13] << 8 | data[12])) * scale_quat;

    msg.linear_acceleration.x = ((int16_t)(data[15] << 8 | data[14])) / 100.0;
    msg.linear_acceleration.y = ((int16_t)(data[17] << 8 | data[16])) / 100.0;
    msg.linear_acceleration.z = ((int16_t)(data[19] << 8 | data[18])) / 100.0;

    msg.angular_velocity.x = ((int16_t)(data[1] << 8 | data[0])) / 900.0;
    msg.angular_velocity.y = ((int16_t)(data[3] << 8 | data[2])) / 900.0;
    msg.angular_velocity.z = ((int16_t)(data[5] << 8 | data[4])) / 900.0;

    double var_o = this->get_parameter("variance_orientation").as_double();
    double var_a = this->get_parameter("variance_angular_velocity").as_double();
    double var_l = this->get_parameter("variance_linear_acceleration").as_double();

    for(int i=0; i<9; i+=4) {
      msg.orientation_covariance[i] = var_o;
      msg.angular_velocity_covariance[i] = var_a;
      msg.linear_acceleration_covariance[i] = var_l;
    }
    
    publisher_->publish(msg);
  }

  int i2c_fd_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BNO055Node>());
  rclcpp::shutdown();
  return 0;
}
