
neonavigation_add_gtest(test_map_organizer SKIP_TEST
  src/test_map_organizer.cpp
  LIBRARIES
    rclcpp::rclcpp
    map_organizer_msgs
    std_msgs
    nav_msgs
)
add_launch_test(launch/map_organizer_test.launch.py TIMEOUT 60)

neonavigation_add_gtest(test_pointcloud_to_maps SKIP_TEST
  src/test_pointcloud_to_maps.cpp
  LIBRARIES
    rclcpp::rclcpp
    map_organizer_msgs
    sensor_msgs
)
add_launch_test(launch/pointcloud_to_maps_test.launch.py TIMEOUT 120)

install(DIRECTORY test data
  DESTINATION share/${PROJECT_NAME}
)

