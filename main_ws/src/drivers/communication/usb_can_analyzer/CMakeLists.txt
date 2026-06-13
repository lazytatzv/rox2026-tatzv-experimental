# Copyright 2026 Tatsukiyano
cmake_minimum_required(VERSION 3.14)
project(seeed_usb_can_analyzer_driver)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(Threads REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/CanFrame.msg"
# )

# 1. Internal protocol library
add_library(usb_can_serial_protocol SHARED
  src/serial_protocol.cpp
)
target_include_directories(usb_can_serial_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(usb_can_serial_protocol Boost::headers)

# 2. Main component library (Shared)
add_library(usb_can_analyzer_component SHARED
  src/usb_can_analyzer_node.cpp
)
target_include_directories(usb_can_analyzer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link all dependencies using plain signature for consistency
ament_target_dependencies(usb_can_analyzer_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  lifecycle_msgs
  custom_interfaces
)

target_link_libraries(usb_can_analyzer_component
  Boost::headers
  usb_can_serial_protocol
)

# 3. Register as Component AND auto-generate standalone executable
rclcpp_components_register_node(usb_can_analyzer_component
  PLUGIN "seeed_usb_can::UsbCanAnalyzerNode"
  EXECUTABLE usb_can_analyzer_node
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  usb_can_serial_protocol
  usb_can_analyzer_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
