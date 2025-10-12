cmake_minimum_required(VERSION 3.14.4)
project(joystick_interrupt)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(joystick_interrupt src/joystick_interrupt.cpp)

ament_auto_add_executable(joystick_mux src/joystick_mux.cpp)


if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  add_subdirectory(test)
endif()


ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE launch)
