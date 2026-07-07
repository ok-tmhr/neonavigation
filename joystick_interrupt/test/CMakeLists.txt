ament_add_gtest(test_joystick_interrupt SKIP_TEST
  src/test_joystick_interrupt.cpp
)
neonavigation_target_link_library(test_joystick_interrupt
  rclcpp::rclcpp
  geometry_msgs
  sensor_msgs
)
add_launch_test(launch/joystick_interrupt_test.launch.py TIMEOUT 120)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)