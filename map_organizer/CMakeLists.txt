cmake_minimum_required(VERSION 3.14.4)
project(map_organizer)

set(AMENT_CMAKE_DEPENDS
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

find_package(ament_cmake REQUIRED)
foreach(DEPEND ${AMENT_CMAKE_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach()

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
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


add_executable(pointcloud_to_maps src/pointcloud_to_maps.cpp)
target_link_libraries(pointcloud_to_maps ${PCL_LIBRARIES})
ament_target_dependencies(pointcloud_to_maps ${AMENT_CMAKE_DEPENDS})

add_executable(tie_maps src/tie_maps.cpp)
target_link_libraries(tie_maps yaml-cpp ${PCL_LIBRARIES})
ament_target_dependencies(tie_maps ${AMENT_CMAKE_DEPENDS})

add_executable(save_maps src/save_maps.cpp)
target_link_libraries(save_maps ${PCL_LIBRARIES})
ament_target_dependencies(save_maps ${AMENT_CMAKE_DEPENDS})

add_executable(select_map src/select_map.cpp)
target_link_libraries(select_map ${PCL_LIBRARIES})
ament_target_dependencies(select_map ${AMENT_CMAKE_DEPENDS})

add_executable(pose_transform src/pose_transform.cpp)
target_link_libraries(pose_transform ${PCL_LIBRARIES})
ament_target_dependencies(pose_transform ${AMENT_CMAKE_DEPENDS})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
    pointcloud_to_maps
    pose_transform
    save_maps
    select_map
    tie_maps
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()