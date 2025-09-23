set(LIBRARIES
  ${diagnostic_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${safety_limiter_msgs_TARGETS}
)

ament_add_gtest(test_safety_limiter
  test/safety_limiter_rostest.test
  src/test_safety_limiter.cpp
)
target_link_libraries(test_safety_limiter ${nav_msgs_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES} ${LIBRARIES})
target_include_directories(test_safety_limiter PUBLIC ${nav_msgs_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})
include_directories(include)

ament_add_gtest(test_safety_limiter2
  test/safety_limiter2_rostest.test
  src/test_safety_limiter2.cpp
)
target_link_libraries(test_safety_limiter2 ${nav_msgs_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES} ${LIBRARIES})
target_include_directories(test_safety_limiter2 PUBLIC ${nav_msgs_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})

install(TARGETS
    test_safety_limiter
    test_safety_limiter2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}/test
)