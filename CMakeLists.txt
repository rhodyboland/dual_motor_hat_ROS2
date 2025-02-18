cmake_minimum_required(VERSION 3.16)
project(dfrobot_dc_motor_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  dfrobot_dc_motor_hardware
  SHARED
  hardware/dfrobot_dc_motor_hardware.cpp
  hardware/dfrobot_dc_motor.cpp
)
target_compile_features(dfrobot_dc_motor_hardware PUBLIC cxx_std_17)
target_include_directories(dfrobot_dc_motor_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/dfrobot_dc_motor_hardware>
)
ament_target_dependencies(
  dfrobot_dc_motor_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# target_link_libraries(dfrobot_dc_motor_hardware PRIVATE i2c)
target_compile_definitions(dfrobot_dc_motor_hardware PRIVATE
  "-DLINUX_I2C_DEV"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DFROBOT_DC_MOTOR_HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface dfrobot_dc_motor_hardware.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/dfrobot_dc_motor_hardware
)
# description/launch
install(
  DIRECTORY description/ros2_control description/urdf description/rviz
  DESTINATION share/dfrobot_dc_motor_hardware
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/dfrobot_dc_motor_hardware
)
install(TARGETS dfrobot_dc_motor_hardware
  EXPORT export_dfrobot_dc_motor_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_dfrobot_dc_motor_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
