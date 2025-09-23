ament_add_gtest(test_obj_to_pointcloud
  test/obj_to_pointcloud_rostest.test
  src/test_obj_to_pointcloud.cpp
)
target_link_libraries(test_obj_to_pointcloud rclcpp::rclcpp ${sensor_msgs_TARGETS})

install(TARGETS test_obj_to_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY test data
  DESTINATION share/${PROJECT_NAME}/test
)