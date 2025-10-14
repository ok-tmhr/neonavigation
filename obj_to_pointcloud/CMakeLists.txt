cmake_minimum_required(VERSION 3.14.4)
project(obj_to_pointcloud)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(obj_to_pointcloud_component src/obj_to_pointcloud.cpp)

rclcpp_components_register_node(
  obj_to_pointcloud_component
  PLUGIN obj_to_pointcloud::ObjToPointcloudNode
  EXECUTABLE obj_to_pointcloud
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package()

