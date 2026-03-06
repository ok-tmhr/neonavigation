cmake_minimum_required(VERSION 3.14.4)
project(safety_limiter)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rclcpp_components_auto_register_node(
  safety_limiter
  PLUGIN safety_limiter::SafetyLimiterNode
  PARAMETERS param/safety_limiter_parameters.yaml
  src/safety_limiter.cpp
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

