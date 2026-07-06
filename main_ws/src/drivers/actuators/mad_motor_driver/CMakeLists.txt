cmake_minimum_required(VERSION 3.8)
project(mad_motor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mad_motor_driver_node src/mad_motor_driver_node.cpp)
ament_target_dependencies(mad_motor_driver_node rclcpp std_msgs can_msgs sensor_msgs)

install(TARGETS mad_motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
