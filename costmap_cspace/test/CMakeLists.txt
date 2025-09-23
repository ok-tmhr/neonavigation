set(LIBRARIES
    rclcpp::rclcpp
    ${costmap_cspace_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
)

ament_add_gtest(test_costmap_3d src/test_costmap_3d.cpp)
target_include_directories(test_costmap_3d PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(test_costmap_3d ${LIBRARIES})

ament_add_gtest(test_pointcloud_accumulator src/test_pointcloud_accumulator.cpp)
target_include_directories(test_pointcloud_accumulator PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(test_pointcloud_accumulator ${LIBRARIES})

install(TARGETS
        test_costmap_3d
        test_pointcloud_accumulator
    DESTINATION lib/${PROJECT_NAME}
)
