cmake_minimum_required(VERSION 3.14.4)
project(trajectory_tracker)

find_package(neonavigation_cmake REQUIRED)

neonavigation_find_build_dependencies(EXCLUDE eigen)
find_package(Eigen3 REQUIRED COMPONENTS Core Geometry)

# Tracker
neonavigation_register_node(trajectory_tracker_component src/trajectory_tracker.cpp
  EXECUTABLE trajectory_tracker
  PLUGIN
    trajectory_tracker::TrackerNode
  PARAMETER
    param/trajectory_tracker_parameters.yaml
  LIBRARIES
    Eigen3::Eigen
    tf2::tf2
    geometry_msgs
    nav_msgs
    tf2_geometry_msgs
    trajectory_tracker_msgs
)
neonavigation_target_include_directories(trajectory_tracker_component PUBLIC)

# Recorder
neonavigation_register_node(trajectory_recorder_component src/trajectory_recorder.cpp
  EXECUTABLE trajectory_recorder
  PLUGIN
    trajectory_tracker::RecorderNode
  LIBRARIES
    geometry_msgs
    nav_msgs
    tf2_geometry_msgs
    std_srvs
)

# Saver
neonavigation_add_executable(trajectory_saver src/trajectory_saver.cpp
  LIBRARIES
    rclcpp::rclcpp
    geometry_msgs
    nav_msgs
)

# Server
neonavigation_register_node(trajectory_server_component src/trajectory_server.cpp
  EXECUTABLE trajectory_server
  PLUGIN trajectory_tracker::ServerNode
  LIBRARIES
    interactive_markers::interactive_markers
    geometry_msgs
    nav_msgs
    tf2_geometry_msgs
    trajectory_tracker_msgs
)
neonavigation_target_include_directories(trajectory_server_component PUBLIC)

neonavigation_add_library(path2d INTERFACE
  LIBRARIES
    Eigen3::Eigen
    geometry_msgs
    nav_msgs
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2::tf2
    rclcpp::rclcpp
)
# target_include_directories(path2d
#    INTERFACE
#       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#       $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )
# neonavigation_target_link_library(path2d
#    INTERFACE
#       Eigen3::Eigen
#       geometry_msgs
#       nav_msgs
#       tf2_geometry_msgs::tf2_geometry_msgs
#       tf2::tf2
#       rclcpp::rclcpp
# )

# install(
#   TARGETS path2d
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(
#   DIRECTORY include
#   DESTINATION include/${PROJECT_NAME}
# )

# install(EXPORT
#   export_${PROJECT_NAME}
#   NAMESPACE ${PROJECT_NAME}::
#   DESTINATION share/${PROJECT_NAME}/cmake
# )

# ament_export_targets(export_${PROJECT_NAME})

# ament_export_dependencies(
#   Eigen3
#   geometry_msgs
#   nav_msgs
#   tf2_geometry_msgs
#   tf2
#   rclcpp
# )

if(BUILD_TESTING)
  neonavigation_find_test_dependencies()
  add_subdirectory(test)
endif()

neonavigation_package(EXPORT_TARGETS)

