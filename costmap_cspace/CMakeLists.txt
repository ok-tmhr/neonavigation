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
  NO_PRIVATE
  LINK_LIBRARIES
    plain_parameters
    unknown_handle_parameters
    footprint_parameters
    costmap_cspace_msgs
    nav_msgs
)

neonavigation_register_node(
  costmap_3d src/costmap_3d.cpp
  PLUGIN costmap_cspace::Costmap3DOFNode
  PARAMETER param/costmap_3d_parameters.yaml
  LINK_LIBRARIES
    costmap_3d_layers
    geometry_msgs
    sensor_msgs
)

neonavigation_register_node(
  laserscan_to_map src/laserscan_to_map.cpp
  PLUGIN costmap_cspace::LaserscanToMapNode
  LINK_LIBRARIES
    laser_geometry::laser_geometry
    nav_msgs
    tf2_geometry_msgs::tf2_geometry_msgs
)

neonavigation_register_node(
  pointcloud2_to_map src/pointcloud2_to_map.cpp
  PLUGIN costmap_cspace::Pointcloud2ToMapNode
  LINK_LIBRARIES
    tf2_geometry_msgs::tf2_geometry_msgs
    nav_msgs
  DEPENDENCIES
    tf2_sensor_msgs
)

neonavigation_register_node(
  largemap_to_map src/largemap_to_map.cpp
  PLUGIN costmap_cspace::LargeMapToMapNode
  LINK_LIBRARIES
    nav_msgs
    tf2_geometry_msgs::tf2_geometry_msgs
)

if(BUILD_TESTING)
  neonavigation_find_test_dependencies()
  add_subdirectory(test)
endif()

neonavigation_package()