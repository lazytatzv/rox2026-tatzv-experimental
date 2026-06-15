cmake_minimum_required(VERSION 3.8)
project(imu_stabilizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
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

# 2. ROS 2 Component Library
add_library(heading_stabilizer_component SHARED src/stabilizer_node.cpp)
target_include_directories(heading_stabilizer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(heading_stabilizer_component imu_stabilizer_core)
ament_target_dependencies(heading_stabilizer_component
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
)
rclcpp_components_register_node(heading_stabilizer_component
  PLUGIN "imu_stabilizer::HeadingStabilizerNode"
  EXECUTABLE stabilizer_node
)

# 3. Dedicated Executable (Wrapper)
add_executable(stabilizer_executable src/stabilizer_executable.cpp)
target_link_libraries(stabilizer_executable heading_stabilizer_component)
ament_target_dependencies(stabilizer_executable rclcpp)

# 4. Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stabilizer_core test/test_stabilizer_core.cpp)
  target_link_libraries(test_stabilizer_core imu_stabilizer_core)
endif()

# 5. Install
install(TARGETS 
  stabilizer_node 
  stabilizer_executable
  imu_stabilizer_core 
  heading_stabilizer_component
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
