cmake_minimum_required(VERSION 3.14.4)
project(costmap_cspace)

find_package(neonavigation_cmake REQUIRED)
neonavigation_find_build_dependencies()

foreach(layer plain unknown_handle footprint)
  generate_parameter_library(
    ${layer}_parameters
    param/costmap_3d_layer/${layer}_parameters.yaml
  )
endforeach()

neonavigation_add_library(costmap_3d_layers src/costmap_3d_layers.cpp
  HAS_HEADER
  LIBRARIES
    plain_parameters
    unknown_handle_parameters
    footprint_parameters
    costmap_cspace_msgs
    nav_msgs
    costmap_cspace_msgs
)

neonavigation_register_node(
  costmap_3d_component src/costmap_3d.cpp
  HAS_HEADER
  EXECUTABLE costmap_3d
  PLUGIN costmap_cspace::Costmap3DOFNode
  PARAMETER param/costmap_3d_parameters.yaml
  LIBRARIES
    costmap_3d_layers
    geometry_msgs
    sensor_msgs
)

neonavigation_register_node(
  laserscan_to_map_component src/laserscan_to_map.cpp
  HAS_HEADER
  EXECUTABLE laserscan_to_map
  PLUGIN costmap_cspace::LaserscanToMapNode
  LIBRARIES
    laser_geometry::laser_geometry
    nav_msgs
    tf2_geometry_msgs::tf2_geometry_msgs
)

neonavigation_register_node(
  pointcloud2_to_map_component src/pointcloud2_to_map.cpp
  HAS_HEADER
  EXECUTABLE pointcloud2_to_map
  PLUGIN costmap_cspace::Pointcloud2ToMapNode
  LIBRARIES
    tf2_geometry_msgs::tf2_geometry_msgs
    nav_msgs
)
ament_target_dependencies(pointcloud2_to_map_component
  tf2_sensor_msgs
)


neonavigation_register_node(
  largemap_to_map_component src/largemap_to_map.cpp
  EXECUTABLE largemap_to_map
  PLUGIN costmap_cspace::LargeMapToMapNode
  LIBRARIES
    nav_msgs
    tf2_geometry_msgs::tf2_geometry_msgs
)

if(BUILD_TESTING)
  neonavigation_find_test_dependencies()
  add_subdirectory(test)
endif()

neonavigation_package()