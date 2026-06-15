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
find_package(seeed_usb_can_analyzer_driver REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# --- Hardware Interface Library ---
add_library(robstride_system_hardware SHARED
  src/robstride_system_hardware.cpp
)
target_include_directories(robstride_system_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robstride_system_hardware
  rclcpp
  hardware_interface
  pluginlib
  std_msgs
  custom_interfaces
  seeed_usb_can_analyzer_driver
)
target_link_libraries(robstride_system_hardware
  Boost::system
)

# --- CLI Tools ---
add_executable(set_motor_id src/set_motor_id.cpp)
target_include_directories(set_motor_id PRIVATE include)
ament_target_dependencies(set_motor_id
  rclcpp
  seeed_usb_can_analyzer_driver
)

# --- Install ---
install(TARGETS
  robstride_system_hardware
  set_motor_id
  DESTINATION lib/${PROJECT_NAME}
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
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
