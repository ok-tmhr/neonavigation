include_directories(include)

ament_add_gtest(test_${PROJECT_NAME}_filter src/test_filter.cpp)
target_link_libraries(test_${PROJECT_NAME}_filter)

ament_add_gtest(test_${PROJECT_NAME}_path2d src/test_path2d.cpp)
target_link_libraries(test_${PROJECT_NAME}_path2d ${nav_msgs_TARGETS} Eigen3::Eigen ${tf2_geometry_msgs_TARGETS} ${trajectory_tracker_msgs_TARGETS})

ament_add_gtest(test_trajectory_recorder test/trajectory_recorder_rostest.test
    src/test_trajectory_recorder.cpp)
target_link_libraries(test_trajectory_recorder rclcpp::rclcpp tf2_ros::tf2_ros ${tf2_geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${std_srvs_TARGETS})

ament_add_gtest(test_trajectory_tracker test/trajectory_tracker_rostest.test
    src/test_trajectory_tracker.cpp)
target_link_libraries(test_trajectory_tracker ${Boost_LIBRARIES} Eigen3::Eigen rclcpp::rclcpp ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${tf2_geometry_msgs_TARGETS} ${trajectory_tracker_msgs_TARGETS})

# add_rostest(test/trajectory_tracker_rostest.test
#   ARGS odom_delay:=0.04 use_odom:=true use_time_optimal_control:=true
#   DEPENDENCIES test_trajectory_tracker
# )

# add_rostest(test/trajectory_tracker_rostest.test
#   ARGS odom_delay:=0.04 use_odom:=true use_time_optimal_control:=false
#   DEPENDENCIES test_trajectory_tracker
# )

ament_add_gtest(test_trajectory_tracker_with_odom test/trajectory_tracker_with_odom_rostest.test
    src/test_trajectory_tracker_with_odom.cpp)
target_link_libraries(test_trajectory_tracker_with_odom ${Boost_LIBRARIES} Eigen3::Eigen rclcpp::rclcpp ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${tf2_geometry_msgs_TARGETS} ${trajectory_tracker_msgs_TARGETS})

ament_add_gtest(test_trajectory_tracker_overshoot test/trajectory_tracker_overshoot_rostest.test
    src/test_trajectory_tracker_overshoot.cpp)
target_link_libraries(test_trajectory_tracker_overshoot ${Boost_LIBRARIES} Eigen3::Eigen rclcpp::rclcpp ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${tf2_geometry_msgs_TARGETS} ${trajectory_tracker_msgs_TARGETS})

install(TARGETS
    test_${PROJECT_NAME}_filter
    test_${PROJECT_NAME}_path2d
    test_trajectory_recorder
    test_trajectory_tracker
    test_trajectory_tracker_with_odom
    test_trajectory_tracker_overshoot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)