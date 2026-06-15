cmake_minimum_required(VERSION 3.8)
project(imu_stabilizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_toolbox REQUIRED)

add_executable(stabilizer_node src/stabilizer_node.cpp)
ament_target_dependencies(stabilizer_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  control_toolbox
)

install(TARGETS stabilizer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
