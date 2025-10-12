cmake_minimum_required(VERSION 3.14.4)
project(safety_limiter)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(
  safety_limiter_parameters
  param/safety_limiter_parameters.yaml
)

ament_auto_add_executable(safety_limiter src/safety_limiter.cpp)
target_link_libraries(safety_limiter safety_limiter_parameters)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

