cmake_minimum_required(VERSION 3.14.4)
project(track_odometry)

find_package(neonavigation_cmake REQUIRED)
neonavigation_find_build_dependencies(EXCLUDE eigen)
find_package(Eigen3 REQUIRED COMPONENTS Core Geometry)

neonavigation_register_node(
  track_odometry_component src/track_odometry.cpp
  HAS_HEADER
  EXECUTABLE track_odometry
  PLUGIN track_odometry::TrackOdometryNode
  PARAMETER param/track_odometry_parameters.yaml
  LIBRARIES
    Eigen3::Eigen
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    message_filters::message_filters
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros
)

neonavigation_register_node(
  tf_projection_component src/tf_projection_node.cpp src/tf_projection.cpp
  HAS_HEADER
  EXECUTABLE tf_projection
  PLUGIN track_odometry::TfProjectionNode
  PARAMETER param/tf_projection_node_parameters.yaml
  LIBRARIES
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros
)

if(BUILD_TESTING)
  neonavigation_find_test_dependencies()
  add_subdirectory(test)
endif()

neonavigation_package()

