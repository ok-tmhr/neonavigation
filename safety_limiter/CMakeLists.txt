cmake_minimum_required(VERSION 3.14.4)
project(safety_limiter)

find_package(neonavigation_cmake REQUIRED)

neonavigation_find_build_dependencies(EXCLUDE eigen)
find_package(PCL REQUIRED COMPONENTS common filters kdtree)
find_package(Eigen3 REQUIRED COMPONENTS Core Geometry)

neonavigation_register_node(
  safety_limiter src/safety_limiter.cpp
  PLUGIN safety_limiter::SafetyLimiterNode
  PARAMETER param/safety_limiter_parameters.yaml
  LINK_LIBRARIES
    Eigen3::Eigen
    rclcpp::rclcpp
    geometry_msgs
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    safety_limiter_msgs
    diagnostic_updater::diagnostic_updater
    pcl_common
    pcl_filters
    pcl_kdtree
  DEPENDENCIES
    pcl_conversions
)

if(BUILD_TESTING)
  neonavigation_find_test_dependencies()
  add_subdirectory(test)
endif()

neonavigation_package()

