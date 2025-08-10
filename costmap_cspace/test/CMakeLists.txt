# catkin_add_gtest(test_costmap_3d src/test_costmap_3d.cpp)
# target_link_libraries(test_costmap_3d ${catkin_LIBRARIES})

ament_add_gtest(test_pointcloud_accumulator src/test_pointcloud_accumulator.cpp)
include_directories(test_pointcloud_accumulator PRIVATE ../include)
target_link_libraries(test_pointcloud_accumulator rclcpp::rclcpp ${sensor_msgs_TARGETS})
