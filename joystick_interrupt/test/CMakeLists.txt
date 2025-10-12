ament_auto_add_gtest(test_joystick_interrupt SKIP_TEST
  src/test_joystick_interrupt.cpp
)
add_launch_test(launch/joystick_interrupt_test.launch.py TIMEOUT 120)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)