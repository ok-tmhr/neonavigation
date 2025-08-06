ament_add_gtest(test_tf_projection
  src/test_tf_projection.cpp
  ../src/tf_projection.cpp
)
target_include_directories(test_tf_projection PRIVATE ../include)
target_link_libraries(test_tf_projection rclcpp::rclcpp tf2::tf2 ${tf2_geometry_msgs_TARGETS})

ament_add_gtest(test_track_odometry
  src/test_track_odometry.cpp
)
# add_rostest_gtest(test_track_odometry
#   test/track_odometry_rostest.test
#   src/test_track_odometry.cpp
# )
target_link_libraries(test_track_odometry rclcpp::rclcpp tf2::tf2 ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${tf2_geometry_msgs_TARGETS})

ament_add_gtest(test_tf_projection_node
  src/test_tf_projection_node.cpp
)
# add_rostest_gtest(test_tf_projection_node
#   test/tf_projection_rostest.test
#   src/test_tf_projection_node.cpp
# )
target_link_libraries(test_tf_projection_node rclcpp::rclcpp tf2::tf2 tf2_ros::tf2_ros ${geometry_msgs_TARGETS})
