cmake_minimum_required(VERSION 3.14.4)
project(map_organizer)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
ament_auto_find_build_dependencies()

include_directories(${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Binary installed pcl provided by Linux distro is built with -march=native
# which causes a lot of compatibility problems.
# Define PCL_NO_PRECOMPILE to disable using the binary version.
add_definitions(-DPCL_NO_PRECOMPILE)

generate_parameter_library(
  pointcloud_to_maps_parameters
  param/pointcloud_to_maps_parameters.yaml
)

ament_auto_add_library(pointcloud_to_maps_component src/pointcloud_to_maps.cpp)
target_link_libraries(pointcloud_to_maps_component ${PCL_LIBRARIES} pointcloud_to_maps_parameters)

ament_auto_add_library(tie_maps_component src/tie_maps.cpp)
target_link_libraries(tie_maps_component yaml-cpp ${PCL_LIBRARIES})

ament_auto_add_executable(save_maps src/save_maps.cpp)
target_link_libraries(save_maps ${PCL_LIBRARIES})

ament_auto_add_library(select_map_component src/select_map.cpp)
target_link_libraries(select_map_component ${PCL_LIBRARIES})

ament_auto_add_library(pose_transform_component src/pose_transform.cpp)
target_link_libraries(pose_transform_component ${PCL_LIBRARIES})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rclcpp_components_register_node(
  pointcloud_to_maps_component
  PLUGIN map_organizer::PointCloudToMaps
  EXECUTABLE pointcloud_to_maps
)

rclcpp_components_register_node(
  tie_maps_component
  PLUGIN map_organizer::TieMapNode
  EXECUTABLE tie_maps
)

rclcpp_components_register_node(
  select_map_component
  PLUGIN map_organizer::SelectMap
  EXECUTABLE select_map
)

rclcpp_components_register_node(
  pose_transform_component
  PLUGIN map_organizer::PoseTransformNode
  EXECUTABLE pose_transform
)

ament_auto_package()

