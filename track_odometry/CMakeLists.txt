cmake_minimum_required(VERSION 3.14.4)
project(track_odometry)

set(AMENT_CMAKE_DEPENDS
  rclcpp

  geometry_msgs
  message_filters
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
foreach(DEPEND ${AMENT_CMAKE_DEPENDS})
  find_package(${DEPEND})
endforeach()
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(track_odometry src/track_odometry.cpp)
target_include_directories(track_odometry PRIVATE include)
ament_target_dependencies(track_odometry ${AMENT_CMAKE_DEPENDS} Eigen3)

add_executable(tf_projection src/tf_projection_node.cpp src/tf_projection.cpp)
target_include_directories(tf_projection PRIVATE include)
ament_target_dependencies(tf_projection ${AMENT_CMAKE_DEPENDS})

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_subdirectory(test)

#   find_package(roslint REQUIRED)
#   roslint_cpp()
#   roslint_add_test()
# endif()


install(TARGETS
    tf_projection
    track_odometry
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()