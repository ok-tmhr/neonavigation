ament_add_gtest(test_costmap_3d src/test_costmap_3d.cpp)
include_directories(test_costmap_3d PRIVATE ../include)
target_link_libraries(test_costmap_3d rclcpp::rclcpp ${costmap_cspace_msgs_TARGETS} ${nav_msgs_TARGETS})

ament_add_gtest(test_pointcloud_accumulator src/test_pointcloud_accumulator.cpp)
include_directories(test_pointcloud_accumulator PRIVATE ../include)
target_link_libraries(test_pointcloud_accumulator rclcpp::rclcpp ${sensor_msgs_TARGETS})
