ament_add_gtest_executable(test_obj_to_pointcloud
  src/test_obj_to_pointcloud.cpp
)
target_link_libraries(test_obj_to_pointcloud rclcpp::rclcpp ${sensor_msgs_TARGETS})
add_launch_test(launch/obj_to_pointcloud_test.launch.py TIMEOUT 120)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}/test
)