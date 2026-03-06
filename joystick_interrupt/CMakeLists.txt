cmake_minimum_required(VERSION 3.14.4)
project(joystick_interrupt)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rclcpp_components_auto_register_node(
  joystick_interrupt
  PLUGIN joystick_interrupt::JoystickInterrupt
  src/joystick_interrupt.cpp
)

rclcpp_components_auto_register_node(
  joystick_mux
  PLUGIN joystick_interrupt::JoystickMux
  src/joystick_mux.cpp
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE launch)
