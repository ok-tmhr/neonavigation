cmake_minimum_required(VERSION 3.14.4)
project(map_organizer)

find_package(neonavigation_cmake REQUIRED)
neonavigation_find_build_dependencies(EXCLUDE eigen)

neonavigation_register_node(
  pointcloud_to_maps
  src/pointcloud_to_maps.cpp
  PLUGIN map_organizer::PointcloudToMapsNode
  PARAMETER param/pointcloud_to_maps_parameters.yaml
  LINK_LIBRARIES
    sensor_msgs
    map_organizer_msgs
    pcl_common
  DEPENDENCIES
    pcl_conversions
)

neonavigation_register_node(
  tie_maps src/tie_maps.cpp
  PLUGIN map_organizer::TieMapNode
  LINK_LIBRARIES
    map_organizer_msgs
  DEPENDENCIES
    nav2_map_server
)

neonavigation_add_executable(save_maps src/save_maps.cpp
  LINK_LIBRARIES
    rclcpp::rclcpp
    nav_msgs
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    map_organizer_msgs
)

neonavigation_register_node(
  select_map src/select_map.cpp
  PLUGIN map_organizer::SelectMap
  LINK_LIBRARIES
    map_organizer_msgs
    tf2_geometry_msgs::tf2_geometry_msgs
)

neonavigation_register_node(
  pose_transform src/pose_transform.cpp
  PLUGIN map_organizer::PoseTransformNode
  LINK_LIBRARIES
    geometry_msgs
    tf2_geometry_msgs::tf2_geometry_msgs
)

if(BUILD_TESTING)
  neonavigation_find_test_dependencies()
  add_subdirectory(test)
endif()

neonavigation_package()
