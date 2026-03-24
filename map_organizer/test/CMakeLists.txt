
ament_auto_add_gtest(test_map_organizer SKIP_TEST
  src/test_map_organizer.cpp
)
add_launch_test(launch/map_organizer_test.launch.py TIMEOUT 60)

ament_auto_add_gtest(test_pointcloud_to_maps SKIP_TEST
  src/test_pointcloud_to_maps.cpp
)
add_launch_test(launch/pointcloud_to_maps_test.launch.py TIMEOUT 120)

install(DIRECTORY test data
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS test_map_organizer test_pointcloud_to_maps
  DESTINATION lib/${PROJECT_NAME}
)