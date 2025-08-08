ament_add_gtest(test_map_organizer
  src/test_map_organizer.cpp
)
target_link_libraries(test_map_organizer rclcpp::rclcpp ${std_msgs_TARGETS} ${map_organizer_msgs_TARGETS})
# add_dependencies(test_map_organizer tie_maps save_maps select_map)

ament_add_gtest(test_pointcloud_to_maps
  src/test_pointcloud_to_maps.cpp
)
target_link_libraries(test_pointcloud_to_maps rclcpp::rclcpp ${map_organizer_msgs_TARGETS} ${sensor_msgs_TARGETS})
# add_dependencies(test_pointcloud_to_maps pointcloud_to_maps)

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}/test
)