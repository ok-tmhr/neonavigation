cmake_minimum_required(VERSION 3.14.4)
project(trajectory_tracker)


find_package(neonavigation_cmake REQUIRED)

neonavigation_find_build_dependencies()

# Tracker
neonavigation_register_node(trajectory_tracker src/trajectory_tracker.cpp
  PLUGIN
    trajectory_tracker::TrackerNode
  PARAMETER
    param/trajectory_tracker_parameters.yaml
  LINK_LIBRARIES
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
  LINK_LIBRARIES
    geometry_msgs
    nav_msgs
    tf2_geometry_msgs
    std_srvs
)

# Saver
neonavigation_add_executable(trajectory_saver src/trajectory_saver.cpp
  LINK_LIBRARIES
    rclcpp::rclcpp
    geometry_msgs
    nav_msgs
)

# Server
neonavigation_register_node(trajectory_server src/trajectory_server.cpp
  PLUGIN trajectory_tracker::ServerNode
  LINK_LIBRARIES
    interactive_markers::interactive_markers
    geometry_msgs
    nav_msgs
    tf2_geometry_msgs
    trajectory_tracker_msgs
)

add_library(path2d INTERFACE)
target_include_directories(path2d
   INTERFACE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
neonavigation_target_link_library(path2d
   INTERFACE
      Eigen3::Eigen
      geometry_msgs
      nav_msgs
      tf2_geometry_msgs::tf2_geometry_msgs
      tf2::tf2
      rclcpp::rclcpp
)

install(
  TARGETS path2d
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)

install(EXPORT
  export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
  Eigen3
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  tf2
  rclcpp
)

if(BUILD_TESTING)
  neonavigation_find_test_dependencies()
  add_subdirectory(test)
endif()

neonavigation_package()

