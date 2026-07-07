
ament_add_gtest(test_costmap_3d src/test_costmap_3d.cpp)
target_link_libraries(test_costmap_3d
   rclcpp::rclcpp
)
target_include_directories(test_costmap_3d PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_add_gtest(test_pointcloud_accumulator src/test_pointcloud_accumulator.cpp)
target_link_libraries(test_pointcloud_accumulator
   rclcpp::rclcpp
   ${sensor_msgs_TARGETS}
)
target_include_directories(test_pointcloud_accumulator PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
