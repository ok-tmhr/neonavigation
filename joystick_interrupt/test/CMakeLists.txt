ament_add_gtest(test_joystick_interrupt
  test/joystick_interrupt_rostest.test
  src/test_joystick_interrupt.cpp
)
target_link_libraries(test_joystick_interrupt rclcpp::rclcpp ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS})

install(TARGETS test_joystick_interrupt
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)