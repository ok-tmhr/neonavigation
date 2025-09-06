cmake_minimum_required(VERSION 3.14.4)
project(safety_limiter)

set(COLCON_DEPENDS
    rclcpp

    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    pcl_conversions
    pcl_ros
    sensor_msgs
    std_msgs
    tf2_ros
    tf2_sensor_msgs
    safety_limiter_msgs

  )

find_package(ament_cmake REQUIRED ${COLCON_DEPENDS})
foreach(COLCON_DEPEND ${COLCON_DEPENDS})
  find_package(${COLCON_DEPEND} REQUIRED)
endforeach()
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/SafetyLimiter.cfg
)

catkin_package(
  COLCON_DEPENDS ${COLCON_DEPENDS}
)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
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


add_executable(safety_limiter src/safety_limiter.cpp)
target_link_libraries(safety_limiter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(safety_limiter ${catkin_EXPORTED_TARGETS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
    safety_limiter
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

