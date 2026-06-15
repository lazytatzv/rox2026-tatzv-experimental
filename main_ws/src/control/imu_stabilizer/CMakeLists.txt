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

# 1. Heading Stabilizer Core Library (Pure Logic)
add_library(imu_stabilizer_core src/heading_stabilizer_core.cpp)
target_include_directories(imu_stabilizer_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(imu_stabilizer_core
  control_toolbox
)

# 2. ROS 2 Node
add_executable(stabilizer_node src/stabilizer_node.cpp)
target_link_libraries(stabilizer_node imu_stabilizer_core)
ament_target_dependencies(stabilizer_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
)

# 3. Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stabilizer_core test/test_stabilizer_core.cpp)
  target_link_libraries(test_stabilizer_core imu_stabilizer_core)
endif()

# 4. Install
install(TARGETS stabilizer_node imu_stabilizer_core
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
