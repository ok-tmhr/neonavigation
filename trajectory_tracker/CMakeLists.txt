cmake_minimum_required(VERSION 3.14.4)
project(trajectory_tracker)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

find_package(ament_cmake_ros REQUIRED)

if(NOT _AMENT_PACKAGE_NAME)
  ament_package_xml()
endif()

foreach(_dep IN LISTS ${PROJECT_NAME}_BUILD_DEPENDS ${PROJECT_NAME}_BUILDTOOL_DEPENDS)
  find_package(${_dep} REQUIRED)
endforeach()

# Tracker
generate_parameter_library(trajectory_tracker_parameters param/trajectory_tracker_parameters.yaml)
add_library(trajectory_tracker_component src/trajectory_tracker.cpp)
target_include_directories(trajectory_tracker_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(trajectory_tracker_component PRIVATE rclcpp::rclcpp rclcpp_components::component Eigen3::Eigen ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} tf2::tf2 ${tf2_geometry_msgs_TARGETS} ${trajectory_tracker_msgs_TARGETS} trajectory_tracker_parameters)
rclcpp_components_register_node(trajectory_tracker_component PLUGIN trajectory_tracker::TrackerNode EXECUTABLE trajectory_tracker)

# Recorder
add_library(trajectory_recorder_component src/trajectory_recorder.cpp)
target_link_libraries(trajectory_recorder_component PRIVATE rclcpp::rclcpp rclcpp_components::component ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${tf2_geometry_msgs_TARGETS} ${std_srvs_TARGETS})
rclcpp_components_register_node(trajectory_recorder_component PLUGIN trajectory_tracker::RecorderNode EXECUTABLE trajectory_recorder)

# Saver
add_executable(trajectory_saver src/trajectory_saver.cpp)
target_link_libraries(trajectory_saver rclcpp::rclcpp ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS})

# Server
add_library(trajectory_server_component src/trajectory_server.cpp)
target_include_directories(trajectory_server_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(trajectory_server_component PRIVATE rclcpp::rclcpp ${geometry_msgs_TARGETS} interactive_markers::interactive_markers ${nav_msgs_TARGETS} ${tf2_geometry_msgs_TARGETS} ${trajectory_tracker_msgs_TARGETS} rclcpp_components::component)
rclcpp_components_register_node(trajectory_server_component PLUGIN trajectory_tracker::ServerNode EXECUTABLE trajectory_server )

if(BUILD_TESTING)
  foreach(_dep IN LISTS ${PROJECT_NAME}_TEST_DEPENDS)
    find_package(${_dep} REQUIRED)
  endforeach()
  add_subdirectory(test)
endif()

install(TARGETS
  trajectory_tracker_component
  trajectory_recorder_component
  trajectory_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

message(=========== ${${PROJECT_NAME}_LIBRARIES} ==========)
ament_package()

