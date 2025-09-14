set(LIBRARIES
  rclcpp::rclcpp
  ${map_organizer_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

ament_add_gtest(test_map_organizer
  test/map_organizer_rostest.test
  src/test_map_organizer.cpp
)
target_link_libraries(test_map_organizer ${LIBRARIES})

ament_add_gtest(test_pointcloud_to_maps
  test/pointcloud_to_maps_rostest.test
  src/test_pointcloud_to_maps.cpp
)
target_link_libraries(test_pointcloud_to_maps ${LIBRARIES})

install(TARGETS
    test_map_organizer
    test_pointcloud_to_maps
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY test data
  DESTINATION share/${PROJECT_NAME}/test
)