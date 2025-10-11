set(LIBRARIES
  ${diagnostic_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${safety_limiter_msgs_TARGETS}
)

ament_add_gtest_executable(test_safety_limiter
  src/test_safety_limiter.cpp
)
target_link_libraries(test_safety_limiter ${nav_msgs_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES} ${LIBRARIES})
target_include_directories(test_safety_limiter PUBLIC ${nav_msgs_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})
include_directories(include)
add_launch_test(launch/safety_limiter_test.launch.py TIMEOUT 120)

ament_add_gtest_executable(test_safety_limiter2
  src/test_safety_limiter2.cpp
)
target_link_libraries(test_safety_limiter2 ${nav_msgs_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES} ${LIBRARIES})
target_include_directories(test_safety_limiter2 PUBLIC ${nav_msgs_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})
add_launch_test(launch/safety_limiter2_test.launch.py TIMEOUT 30)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)