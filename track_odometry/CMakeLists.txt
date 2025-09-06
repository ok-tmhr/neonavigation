cmake_minimum_required(VERSION 3.14.4)
project(track_odometry)

set(COLCON_DEPENDS
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

find_package(ament_cmake REQUIRED ${COLCON_DEPENDS})
foreach(COLCON_DEPEND ${COLCON_DEPENDS})
  find_package(${COLCON_DEPEND} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)
catkin_package(
  COLCON_DEPENDS ${COLCON_DEPENDS}
)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


add_executable(track_odometry src/track_odometry.cpp)
target_link_libraries(track_odometry ${catkin_LIBRARIES})
add_dependencies(track_odometry ${catkin_EXPORTED_TARGETS})

add_executable(tf_projection src/tf_projection_node.cpp src/tf_projection.cpp)
target_link_libraries(tf_projection ${catkin_LIBRARIES})
add_dependencies(tf_projection ${catkin_EXPORTED_TARGETS})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
    tf_projection
    track_odometry
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

