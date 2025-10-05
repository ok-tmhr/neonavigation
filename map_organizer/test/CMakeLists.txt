set(LIBRARIES
  rclcpp::rclcpp
  ${map_organizer_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

ament_add_gtest_executable(test_map_organizer
  src/test_map_organizer.cpp
)
target_link_libraries(test_map_organizer ${LIBRARIES})
add_launch_test(launch/map_organizer_test.launch.py TIMEOUT 60)

ament_add_gtest_executable(test_pointcloud_to_maps
  src/test_pointcloud_to_maps.cpp
)
target_link_libraries(test_pointcloud_to_maps ${LIBRARIES})
add_launch_test(launch/pointcloud_to_maps_test.launch.py TIMEOUT 120)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}/test
)
