
ament_add_gtest(test_map_organizer SKIP_TEST
  src/test_map_organizer.cpp
)
neonavigation_target_link_library(test_map_organizer
  rclcpp::rclcpp
  map_organizer_msgs
  std_msgs
  nav_msgs
)
add_launch_test(launch/map_organizer_test.launch.py TIMEOUT 60)

ament_add_gtest(test_pointcloud_to_maps SKIP_TEST
  src/test_pointcloud_to_maps.cpp
)
neonavigation_target_link_library(test_pointcloud_to_maps
  rclcpp::rclcpp
  map_organizer_msgs
  sensor_msgs
)
add_launch_test(launch/pointcloud_to_maps_test.launch.py TIMEOUT 120)

install(DIRECTORY test data
  DESTINATION share/${PROJECT_NAME}
)

