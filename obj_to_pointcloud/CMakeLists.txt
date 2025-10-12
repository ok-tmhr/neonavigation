cmake_minimum_required(VERSION 3.14.4)
project(obj_to_pointcloud)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Binary installed pcl provided by Linux distro is built with -march=native
# which causes a lot of compatibility problems.
# Define PCL_NO_PRECOMPILE to disable using the binary version.
add_definitions(-DPCL_NO_PRECOMPILE)


ament_auto_add_executable(obj_to_pointcloud src/obj_to_pointcloud.cpp)


if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package()

