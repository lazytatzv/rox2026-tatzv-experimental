cmake_minimum_required(VERSION 3.8)
project(mad_motor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

add_library(mad_motor_command_node SHARED
  src/mad_motor_command_node.cpp
)
target_include_directories(mad_motor_command_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mad_motor_command_node
  rclcpp::rclcpp
  std_msgs::std_msgs
  can_msgs::can_msgs
)
rclcpp_components_register_node(mad_motor_command_node
  PLUGIN "mad_motor_driver::MadMotorCommandNode"
  EXECUTABLE mad_motor_command_node_exe
)

install(TARGETS
  mad_motor_command_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  can_msgs
)

ament_package()
