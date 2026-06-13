cmake_minimum_required(VERSION 3.8)
project(robstride_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(diagnostic_updater REQUIRED)

# --- Hardware Interface Library ---
add_library(robstride_system_hardware SHARED
  src/robstride_system_hardware.cpp
)
target_include_directories(robstride_system_hardware PRIVATE
  include
)
ament_target_dependencies(robstride_system_hardware
  rclcpp
  hardware_interface
  pluginlib
  std_msgs
  custom_interfaces
)

# --- Install ---
install(TARGETS
  robstride_system_hardware
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  robstride_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface robstride_hardware.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
