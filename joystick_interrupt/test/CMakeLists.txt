ament_add_gtest_executable(test_joystick_interrupt
  src/test_joystick_interrupt.cpp
)
target_link_libraries(test_joystick_interrupt rclcpp::rclcpp ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS})
add_launch_test(launch/joystick_interrupt_test.launch.py TIMEOUT 120)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)