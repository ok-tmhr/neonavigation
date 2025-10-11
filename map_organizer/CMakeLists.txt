cmake_minimum_required(VERSION 3.14.4)
project(map_organizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wshadow)
endif()

set(COLCON_DEPENDS
  rclcpp

  geometry_msgs
  nav2_map_server
  nav_msgs
  pcl_conversions
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros

  map_organizer_msgs
)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
ament_auto_find_build_dependencies()

include_directories(${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Workaround for debian stretch build (https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=894656)
list(REMOVE_ITEM PCL_LIBRARIES
  "vtkGUISupportQt"
  "vtkGUISupportQtOpenGL"
  "vtkGUISupportQtSQL"
  "vtkGUISupportQtWebkit"
  "vtkViewsQt"
  "vtkRenderingQt"
)

# Workaround for the bug in PCL(<1.8.1) https://github.com/PointCloudLibrary/pcl/issues/1406
remove_definitions(-DDISABLE_LIBUSB-1.0)

# Binary installed pcl provided by Linux distro is built with -march=native
# which causes a lot of compatibility problems.
# Define PCL_NO_PRECOMPILE to disable using the binary version.
add_definitions(-DPCL_NO_PRECOMPILE)

generate_parameter_library(
  pointcloud_to_maps_parameters
  param/pointcloud_to_maps_parameters.yaml
)
add_library(pointcloud_to_maps_component SHARED src/pointcloud_to_maps.cpp)
target_include_directories(pointcloud_to_maps_component PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(pointcloud_to_maps_component ${PCL_LIBRARIES} pointcloud_to_maps_parameters)
ament_target_dependencies(pointcloud_to_maps_component ${COLCON_DEPENDS} rclcpp_components)

ament_auto_add_executable(tie_maps src/tie_maps.cpp)
target_link_libraries(tie_maps yaml-cpp ${PCL_LIBRARIES})

ament_auto_add_executable(save_maps src/save_maps.cpp)
target_link_libraries(save_maps ${PCL_LIBRARIES})

ament_auto_add_executable(select_map src/select_map.cpp)
target_link_libraries(select_map ${PCL_LIBRARIES})

ament_auto_add_executable(pose_transform src/pose_transform.cpp)
target_link_libraries(pose_transform ${PCL_LIBRARIES})


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

ament_export_targets(export_pointcloud_to_maps_component HAS_LIBRARY_TARGET)
install(TARGETS pointcloud_to_maps_component pointcloud_to_maps_parameters
  EXPORT export_pointcloud_to_maps_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_auto_package()

