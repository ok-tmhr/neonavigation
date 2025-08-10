ament_add_gtest(test_obj_to_pointcloud
  src/test_obj_to_pointcloud.cpp
)
target_link_libraries(test_obj_to_pointcloud rclcpp::rclcpp ${sensor_msgs_TARGETS})
