add_library(test_safety_limiter_base INTERFACE)
neonavigation_target_link_libraries(
  test_safety_limiter_base
  INTERFACE
    safety_limiter_msgs
    tf2_geometry_msgs
    tf2_ros::tf2_ros
    std_msgs
)

ament_add_gtest(test_safety_limiter SKIP_TEST
  src/test_safety_limiter.cpp
)
neonavigation_target_link_libraries(test_safety_limiter
  rclcpp::rclcpp
  diagnostic_msgs
  geometry_msgs
  sensor_msgs
  test_safety_limiter_base
)
target_include_directories(test_safety_limiter PUBLIC include)
add_launch_test(launch/safety_limiter_test.launch.py TIMEOUT 120)

ament_add_gtest(test_safety_limiter2 SKIP_TEST
src/test_safety_limiter2.cpp
)
neonavigation_target_link_libraries(test_safety_limiter2
  rclcpp::rclcpp
  diagnostic_msgs
  geometry_msgs
  sensor_msgs
  test_safety_limiter_base
)
target_include_directories(test_safety_limiter2 PUBLIC include)
add_launch_test(launch/safety_limiter2_test.launch.py TIMEOUT 30)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)