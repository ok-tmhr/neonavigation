cmake_minimum_required(VERSION 3.14.4)
project(map_organizer)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(
  pointcloud_to_maps_parameters
  param/pointcloud_to_maps_parameters.yaml
)

ament_auto_add_library(pointcloud_to_maps_component src/pointcloud_to_maps.cpp)
target_link_libraries(pointcloud_to_maps_component pointcloud_to_maps_parameters)

rclcpp_components_register_node(
  pointcloud_to_maps_component
  PLUGIN map_organizer::PointcloudToMapsNode
  EXECUTABLE pointcloud_to_maps
)

ament_auto_add_library(tie_maps_component src/tie_maps.cpp)

rclcpp_components_register_node(
  tie_maps_component
  PLUGIN map_organizer::TieMapNode
  EXECUTABLE tie_maps
)

ament_auto_add_executable(save_maps src/save_maps.cpp)

ament_auto_add_library(select_map_component src/select_map.cpp)

rclcpp_components_register_node(
  select_map_component
  PLUGIN map_organizer::SelectMap
  EXECUTABLE select_map
)

ament_auto_add_library(pose_transform_component src/pose_transform.cpp)

rclcpp_components_register_node(
  pose_transform_component
  PLUGIN map_organizer::PoseTransformNode
  EXECUTABLE pose_transform
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

