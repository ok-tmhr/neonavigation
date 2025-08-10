ament_add_gtest(test_safety_limiter
  src/test_safety_limiter.cpp
)
target_link_libraries(test_safety_limiter rclcpp::rclcpp ${nav_msgs_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES} ${diagnostic_msgs_TARGETS} ${sensor_msgs_TARGETS} ${safety_limiter_msgs_TARGETS})
target_include_directories(test_safety_limiter PUBLIC ${nav_msgs_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})
include_directories(include)


ament_add_gtest(test_safety_limiter2
  src/test_safety_limiter2.cpp
)
target_link_libraries(test_safety_limiter2 rclcpp::rclcpp ${nav_msgs_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES} ${diagnostic_msgs_TARGETS} ${sensor_msgs_TARGETS} ${safety_limiter_msgs_TARGETS})
target_include_directories(test_safety_limiter2 PUBLIC ${nav_msgs_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})
