cmake_minimum_required(VERSION 3.14.4)
project(obj_to_pointcloud)

find_package(neonavigation_cmake REQUIRED)
neonavigation_find_build_dependencies(EXCLUDE eigen)
find_package(PCL COMPONENTS filters io)

neonavigation_register_node(
  obj_to_pointcloud src/obj_to_pointcloud.cpp
  PLUGIN obj_to_pointcloud::ObjToPointcloudNode
  LINK_LIBRARIES
    sensor_msgs
    pcl_filters
    pcl_io
  DEPENDENCIES
    pcl_conversions
)

if(BUILD_TESTING)
  neonavigation_find_test_dependencies()
  add_subdirectory(test)
endif()

neonavigation_package()

