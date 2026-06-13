cmake_minimum_required(VERSION 3.8)
project(virtual_actuator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Hardware Interface
add_library(virtual_system_hardware SHARED
  src/virtual_system_hardware.cpp
)
target_include_directories(virtual_system_hardware PRIVATE
  include
)
ament_target_dependencies(virtual_system_hardware
  rclcpp
  hardware_interface
  pluginlib
)

# Install
install(TARGETS
  virtual_system_hardware
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  virtual_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface virtual_hardware.xml)

ament_package()
