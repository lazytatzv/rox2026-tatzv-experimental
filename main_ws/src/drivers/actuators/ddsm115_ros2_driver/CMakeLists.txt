# Copyright 2026 Tatsukiyano
cmake_minimum_required(VERSION 3.8)
project(ddsm115_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Boost REQUIRED CONFIG COMPONENTS headers)
find_package(rosidl_default_generators REQUIRED)

# Message files
set(msg_files
  "msg/Ddsm115Command.msg"
  "msg/Ddsm115Status.msg"
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Add library for component support
add_library(ddsm115_driver_component SHARED
  src/ddsm115_ros2_driver_node.cpp
  src/ddsm115_ros2_driver_client.cpp
)

target_include_directories(ddsm115_driver_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(ddsm115_driver_component PUBLIC cxx_std_20)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

ament_target_dependencies(ddsm115_driver_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  sensor_msgs
  diagnostic_updater
)

target_link_libraries(ddsm115_driver_component
  Boost::headers
  "${cpp_typesupport_target}"
)

# Register as component
rclcpp_components_register_node(ddsm115_driver_component
  PLUGIN "ddsm115_ros2_driver::DDSM115DriverNode"
  EXECUTABLE ddsm115_driver_node
)

# Install targets
install(TARGETS
  ddsm115_driver_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install scripts files
install(PROGRAMS scripts/set_motor_id.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
