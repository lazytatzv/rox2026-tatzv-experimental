cmake_minimum_required(VERSION 3.8)
project(bno055_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(rclcpp_components REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_library(bno055_component SHARED src/bno055_node.cpp)
target_include_directories(bno055_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bno055_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  tf2_ros
)
rclcpp_components_register_node(bno055_component PLUGIN "bno055_driver::BNO055Node" EXECUTABLE bno055_node)

install(TARGETS bno055_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS bno055_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
