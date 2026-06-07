cmake_minimum_required(VERSION 3.8)
project(mecanum_kinematics)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(diagnostic_updater REQUIRED)

# kinematics implementation (Internal library)
add_library(mecanum_kinematics_kinematics SHARED
  src/kinematics.cpp
)
target_include_directories(mecanum_kinematics_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mecanum_kinematics_kinematics
  rclcpp
)

# Main component
add_library(mecanum_kinematics_component SHARED
  src/mecanum_kinematics_node.cpp
  src/wheel_speeds_dispatcher.cpp
)
target_include_directories(mecanum_kinematics_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Crucial: Order and completeness for Jazzy/Humble
ament_target_dependencies(mecanum_kinematics_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  rcl_interfaces
  diagnostic_updater
)

target_link_libraries(mecanum_kinematics_component
  mecanum_kinematics_kinematics
)

rclcpp_components_register_node(mecanum_kinematics_component
  PLUGIN "mecanum_kinematics::MecanumKinematicsNode"
  EXECUTABLE mecanum_kinematics_node
)
rclcpp_components_register_node(mecanum_kinematics_component
  PLUGIN "mecanum_kinematics::WheelSpeedsDispatcher"
  EXECUTABLE wheel_speeds_dispatcher
)

add_executable(zero_twist_node src/zero_twist_node.cpp)
ament_target_dependencies(zero_twist_node rclcpp geometry_msgs)

add_executable(odom_to_tf_node src/odom_to_tf_node.cpp)
ament_target_dependencies(odom_to_tf_node rclcpp nav_msgs tf2_ros)

install(TARGETS
  mecanum_kinematics_component
  mecanum_kinematics_kinematics
  zero_twist_node
  odom_to_tf_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  if(TARGET test_kinematics)
    target_include_directories(test_kinematics PRIVATE include)
    target_link_libraries(test_kinematics mecanum_kinematics_kinematics)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
