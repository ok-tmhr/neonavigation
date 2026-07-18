cmake_minimum_required(VERSION 3.14.4)
project(map_organizer)

find_package(neonavigation_cmake REQUIRED)
neonavigation_find_build_dependencies(EXCLUDE libpcl-all-dev)
find_package(PCL REQUIRED COMPONENTS common)

neonavigation_register_node(
  pointcloud_to_maps_component
  src/pointcloud_to_maps.cpp
  EXECUTABLE pointcloud_to_maps
  PLUGIN map_organizer::PointcloudToMapsNode
  PARAMETER param/pointcloud_to_maps_parameters.yaml
  LIBRARIES
    sensor_msgs
    map_organizer_msgs
    pcl_common
)
ament_target_dependencies(pointcloud_to_maps_component
  pcl_conversions
)

neonavigation_register_node(
  tie_maps_component src/tie_maps.cpp
  EXECUTABLE tie_maps
  PLUGIN map_organizer::TieMapNode
  PARAMETER param/tie_maps_parameters.yaml
  LIBRARIES
    map_organizer_msgs
)
ament_target_dependencies(tie_maps_component
  nav2_map_server
)

neonavigation_add_executable(save_maps src/save_maps.cpp
  LIBRARIES
    rclcpp::rclcpp
    nav_msgs
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    map_organizer_msgs
)

neonavigation_register_node(
  select_map_component src/select_map.cpp
  EXECUTABLE select_map
  PLUGIN map_organizer::SelectMap
  LIBRARIES
    map_organizer_msgs
    tf2_geometry_msgs::tf2_geometry_msgs
)

neonavigation_register_node(
  pose_transform_component src/pose_transform.cpp
  EXECUTABLE pose_transform
  PLUGIN map_organizer::PoseTransformNode
  LIBRARIES
    geometry_msgs
    tf2_geometry_msgs::tf2_geometry_msgs
)

if(BUILD_TESTING)
  neonavigation_find_test_dependencies()
  add_subdirectory(test)
endif()

neonavigation_package()
