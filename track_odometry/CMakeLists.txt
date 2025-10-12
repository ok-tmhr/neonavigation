cmake_minimum_required(VERSION 3.14.4)
project(track_odometry)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(
  track_odometry_parameters
  param/track_odometry_parameters.yaml
)

ament_auto_add_library(track_odometry_component src/track_odometry.cpp)
target_link_libraries(track_odometry_component track_odometry_parameters)

rclcpp_components_register_node(
  track_odometry_component
  PLUGIN track_odometry::TrackOdometryNode
  EXECUTABLE track_odometry
)

generate_parameter_library(
  tf_projection_parameters
  param/tf_projection_node_parameters.yaml
)

ament_auto_add_library(tf_projection_component src/tf_projection_node.cpp src/tf_projection.cpp)
target_link_libraries(tf_projection_component tf_projection_parameters)

rclcpp_components_register_node(
  tf_projection_component
  PLUGIN track_odometry::TfProjectionNode
  EXECUTABLE tf_projection
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

