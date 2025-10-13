cmake_minimum_required(VERSION 3.14.4)
project(trajectory_tracker)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rclcpp_components_auto_register_node(
  trajectory_tracker
  PLUGIN trajectory_tracker::TrackerNode
  PARAMETERS param/trajectory_tracker_parameters.yaml
  src/trajectory_tracker.cpp
)

rclcpp_components_auto_register_node(
  trajectory_recorder
  PLUGIN trajectory_tracker::RecorderNode
  src/trajectory_recorder.cpp
)

ament_auto_add_executable(trajectory_saver src/trajectory_saver.cpp)

rclcpp_components_auto_register_node(
  trajectory_server
  PLUGIN trajectory_tracker::ServerNode
  src/trajectory_server.cpp
)


if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  add_subdirectory(test)
endif()


ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

