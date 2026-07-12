cmake_minimum_required(VERSION 3.14.4)
project(obj_to_pointcloud)

find_package(neonavigation_cmake REQUIRED)
neonavigation_find_build_dependencies(EXCLUDE eigen libpcl-all-dev)
find_package(PCL COMPONENTS filters io)

neonavigation_register_node(
  ${PROJECT_NAME}_component src/${PROJECT_NAME}.cpp
  EXECUTABLE ${PROJECT_NAME}
  PLUGIN ${PROJECT_NAME}::ObjToPointcloudNode
  LIBRARIES
    sensor_msgs
    pcl_filters
    pcl_io
)
ament_target_dependencies(${PROJECT_NAME}_component
  pcl_conversions
)

if(BUILD_TESTING)
  neonavigation_find_test_dependencies()
  add_subdirectory(test)
endif()

neonavigation_package()

