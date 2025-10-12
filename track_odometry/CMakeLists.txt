cmake_minimum_required(VERSION 3.14.4)
project(track_odometry)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rclcpp_components_auto_register_node(
  track_odometry
  PLUGIN track_odometry::TrackOdometryNode
  PARAMETERS param/track_odometry_parameters.yaml
  src/track_odometry.cpp
)

rclcpp_components_auto_register_node(
  tf_projection
  PLUGIN track_odometry::TfProjectionNode
  PARAMETERS param/tf_projection_node_parameters.yaml
  src/tf_projection_node.cpp src/tf_projection.cpp
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

