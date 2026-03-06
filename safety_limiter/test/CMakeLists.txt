
ament_auto_add_gtest(test_safety_limiter SKIP_TEST
  src/test_safety_limiter.cpp
)
add_launch_test(launch/safety_limiter_test.launch.py TIMEOUT 120)

ament_auto_add_gtest(test_safety_limiter2 SKIP_TEST
  src/test_safety_limiter2.cpp
)
add_launch_test(launch/safety_limiter2_test.launch.py TIMEOUT 30)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)