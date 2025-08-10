cmake_minimum_required(VERSION 3.14.4)
project(obj_to_pointcloud)

set(AMENT_CMAKE_DEPENDS
  rclcpp

  geometry_msgs
  pcl_conversions
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
foreach(DEPEND ${AMENT_CMAKE_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach()

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

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


add_executable(obj_to_pointcloud src/obj_to_pointcloud.cpp)
target_link_libraries(obj_to_pointcloud ${PCL_LIBRARIES})
ament_target_dependencies(obj_to_pointcloud ${AMENT_CMAKE_DEPENDS})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
    obj_to_pointcloud
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()