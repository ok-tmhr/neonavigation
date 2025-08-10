ament_add_gtest(test_joystick_interrupt
  src/test_joystick_interrupt.cpp
)
target_link_libraries(test_joystick_interrupt rclcpp::rclcpp ${sensor_msgs_TARGETS})
