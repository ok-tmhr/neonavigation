cmake_minimum_required(VERSION 3.14.4)
project(track_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wshadow)
endif()

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

find_package(ament_cmake REQUIRED)
foreach(COLCON_DEPEND ${COLCON_DEPENDS})
  find_package(${COLCON_DEPEND} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(track_odometry src/track_odometry.cpp)
target_include_directories(track_odometry PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(track_odometry)
ament_target_dependencies(track_odometry ${COLCON_DEPENDS})

add_executable(tf_projection src/tf_projection_node.cpp src/tf_projection.cpp)
target_include_directories(tf_projection PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(tf_projection)
ament_target_dependencies(tf_projection ${COLCON_DEPENDS})


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

