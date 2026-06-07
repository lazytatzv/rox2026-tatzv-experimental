// Copyright 2026 Tatsukiyano
cmake_minimum_required(VERSION 3.8)
project(base_teleop)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(base_teleop_component SHARED
  src/base_teleop_node.cpp
)
target_include_directories(base_teleop_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(base_teleop_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  rcl_interfaces
)

# Register as Component AND auto-generate standalone executable
rclcpp_components_register_node(base_teleop_component
  PLUGIN "base_teleop::BaseTeleopNode"
  EXECUTABLE base_teleop_node
)

# Crucial: Install targets so they can be found at runtime
install(TARGETS
  base_teleop_component
  DESTINATION lib
)

install(TARGETS
  base_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
