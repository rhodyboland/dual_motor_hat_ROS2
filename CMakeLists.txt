cmake_minimum_required(VERSION 3.5)
project(dfrobot_dc_motor_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) # Added
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Declare a C++ library
add_library(dfrobot_dc_motor_hardware SHARED
  src/dfrobot_dc_motor_hardware.cpp
  src/i2c_interface.cpp
)

# Link dependencies
ament_target_dependencies(dfrobot_dc_motor_hardware
  rclcpp
  hardware_interface
  pluginlib
  ros2_control
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs # Added
  geometry_msgs
  std_msgs
)

# Specify additional include directories for the library
target_include_directories(dfrobot_dc_motor_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install the library
install(TARGETS dfrobot_dc_motor_hardware
  DESTINATION lib/${PROJECT_NAME}
)

# Install plugin description file
install(
  DIRECTORY plugin_description
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install configuration files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin descriptions
pluginlib_export_plugin_description_file(hardware_interface plugin_description/hardware_plugins.xml)

# Package
ament_package()
