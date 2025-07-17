cmake_minimum_required(VERSION 3.8)
project(isaac_system_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(isaac_system_interface SHARED
  src/isaac_system_interface.cpp
)
target_include_directories(isaac_system_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(isaac_system_interface
  rclcpp
  hardware_interface
  controller_interface
  pluginlib
  realtime_tools
  sensor_msgs
)

pluginlib_export_plugin_description_file(hardware_interface isaac_system_interface_plugin.xml)

install(
  TARGETS isaac_system_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES isaac_system_interface_plugin.xml
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
