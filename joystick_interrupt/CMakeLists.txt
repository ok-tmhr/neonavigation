cmake_minimum_required(VERSION 3.14.4)
project(joystick_interrupt)

find_package(neonavigation_cmake REQUIRED)
neonavigation_find_build_dependencies()

neonavigation_register_node(
  joystick_interrupt_component src/joystick_interrupt.cpp
  EXECUTABLE joystick_interrupt
  PLUGIN joystick_interrupt::JoystickInterrupt
  LIBRARIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

neonavigation_register_node(
  joystick_mux_component src/joystick_mux.cpp
  EXECUTABLE joystick_mux
  PLUGIN joystick_interrupt::JoystickMux
  LIBRARIES
    sensor_msgs
)

if(BUILD_TESTING)
  neonavigation_find_test_dependencies()
  add_subdirectory(test)
endif()

neonavigation_package(INSTALL_TO_SHARE launch)
