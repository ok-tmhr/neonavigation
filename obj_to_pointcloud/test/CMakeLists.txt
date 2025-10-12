ament_auto_add_gtest(test_obj_to_pointcloud SKIP_TEST
  src/test_obj_to_pointcloud.cpp
)
add_launch_test(launch/obj_to_pointcloud_test.launch.py TIMEOUT 120)

install(DIRECTORY test data
  DESTINATION share/${PROJECT_NAME}
)
