cmake_minimum_required(VERSION 3.14.4)
project(costmap_cspace)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(costmap_3d_layers src/costmap_3d_layers.cpp)

generate_parameter_library(
  plan_parameters
  param/costmap_3d_layer/plain_parameters.yaml
)

generate_parameter_library(
  unknown_handle_parameters
  param/costmap_3d_layer/unknown_handle_parameters.yaml
)

rclcpp_components_auto_register_node(
  costmap_3d
  PLUGIN costmap_cspace::Costmap3DOFNode
  src/costmap_3d.cpp
)
target_link_libraries(costmap_3d_component costmap_3d_layers)

rclcpp_components_auto_register_node(
  laserscan_to_map
  PLUGIN costmap_cspace::LaserscanToMapNode
  src/laserscan_to_map.cpp
)

rclcpp_components_auto_register_node(
  pointcloud2_to_map
  PLUGIN costmap_cspace::Pointcloud2ToMapNode
  src/pointcloud2_to_map.cpp
)

rclcpp_components_auto_register_node(
  largemap_to_map
  PLUGIN costmap_cspace::LargeMapToMapNode
  src/largemap_to_map.cpp
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
