
ament_add_gtest(test_costmap_3d src/test_costmap_3d.cpp)
neonavigation_target_link_libraries(test_costmap_3d
   rclcpp::rclcpp
   costmap_cspace_msgs
   nav_msgs
       plain_parameters
    unknown_handle_parameters
    footprint_parameters
    costmap_cspace_msgs
)
target_include_directories(test_costmap_3d PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_add_gtest(test_pointcloud_accumulator src/test_pointcloud_accumulator.cpp)
neonavigation_target_link_libraries(test_pointcloud_accumulator
   rclcpp::rclcpp
   sensor_msgs
)
target_include_directories(test_pointcloud_accumulator PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
