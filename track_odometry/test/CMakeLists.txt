set(LIBRARIES
  rclcpp::rclcpp
  tf2::tf2
  ${tf2_geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

ament_add_gtest(test_tf_projection
  src/test_tf_projection.cpp
  ../src/tf_projection.cpp
)
target_link_libraries(test_tf_projection ${LIBRARIES})
target_include_directories(test_tf_projection PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_add_gtest_executable(test_track_odometry
  src/test_track_odometry.cpp
)
target_link_libraries(test_track_odometry ${LIBRARIES})
add_launch_test(launch/track_odometry_test.launch.py TIMEOUT 60)

ament_add_gtest_executable(test_tf_projection_node
  src/test_tf_projection_node.cpp
)
target_link_libraries(test_tf_projection_node ${LIBRARIES})
add_launch_test(launch/tf_projection_test.launch.py TIMEOUT 60)

install(TARGETS
    test_track_odometry
    test_tf_projection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)