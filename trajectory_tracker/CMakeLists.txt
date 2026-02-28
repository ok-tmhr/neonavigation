cmake_minimum_required(VERSION 3.14.4)
project(trajectory_tracker)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

find_package(neonavigation_cmake REQUIRED)

neonavigation_find_build_dependencies()

# Tracker
neonavigation_register_node(trajectory_tracker src/trajectory_tracker.cpp
  PLUGIN
    trajectory_tracker::TrackerNode
  PARAMETER
    param/trajectory_tracker_parameters.yaml
  LINK
    Eigen3::Eigen
    tf2::tf2
    geometry_msgs
    nav_msgs
    tf2_geometry_msgs
    trajectory_tracker_msgs
)

# Recorder
neonavigation_register_node(trajectory_recorder src/trajectory_recorder.cpp
  PLUGIN
    trajectory_tracker::RecorderNode
  NO_TARGET_INCLUDE_DIRECTORIES
  LINK
    geometry_msgs
    nav_msgs
    tf2_geometry_msgs
    std_srvs
)

# Saver
neonavigation_add_executable(trajectory_saver src/trajectory_saver.cpp
  LINK
    rclcpp::rclcpp
    geometry_msgs
    nav_msgs
)

# Server
neonavigation_register_node(trajectory_server src/trajectory_server.cpp
  PLUGIN trajectory_tracker::ServerNode
  LINK
    interactive_markers::interactive_markers
    geometry_msgs
    nav_msgs
    tf2_geometry_msgs
    trajectory_tracker_msgs
)

if(BUILD_TESTING)
  neonavigation_find_test_dependencies()
  add_subdirectory(test)
endif()

neonavigation_package()

