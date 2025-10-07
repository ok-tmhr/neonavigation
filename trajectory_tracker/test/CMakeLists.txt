include_directories(include)

ament_add_gtest(test_${PROJECT_NAME}_filter src/test_filter.cpp)
target_link_libraries(test_${PROJECT_NAME}_filter)

ament_add_gtest(test_${PROJECT_NAME}_path2d src/test_path2d.cpp)
target_link_libraries(test_${PROJECT_NAME}_path2d ${nav_msgs_TARGETS} Eigen3::Eigen ${tf2_geometry_msgs_TARGETS} ${trajectory_tracker_msgs_TARGETS})

ament_add_gtest_executable(test_trajectory_recorder
    src/test_trajectory_recorder.cpp)
target_link_libraries(test_trajectory_recorder rclcpp::rclcpp tf2_ros::tf2_ros ${tf2_geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${std_srvs_TARGETS})
add_launch_test(launch/trajectory_recorder_test.launch.py TIMEOUT 60)

ament_add_gtest_executable(test_trajectory_tracker
    src/test_trajectory_tracker.cpp)
target_link_libraries(test_trajectory_tracker Eigen3::Eigen rclcpp::rclcpp ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${tf2_geometry_msgs_TARGETS} ${trajectory_tracker_msgs_TARGETS})

add_launch_test(launch/trajectory_tracker_test.launch.py
  ARGS odom_delay:=0.04 use_odom:=true use_time_optimal_control:=true
  TARGET test_trajectory_tracker_rostest1
  TIMEOUT 120
)

add_launch_test(launch/trajectory_tracker_test.launch.py
  ARGS odom_delay:=0.04 use_odom:=true use_time_optimal_control:=false
  TARGET test_trajectory_tracker_rostest2
  TIMEOUT 120
)

ament_add_gtest_executable(test_trajectory_tracker_with_odom
    src/test_trajectory_tracker_with_odom.cpp)
target_link_libraries(test_trajectory_tracker_with_odom Eigen3::Eigen rclcpp::rclcpp ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${tf2_geometry_msgs_TARGETS} ${trajectory_tracker_msgs_TARGETS})
add_launch_test(launch/trajectory_tracker_with_odom_test.launch.py TIMEOUT 120)

ament_add_gtest_executable(test_trajectory_tracker_overshoot
    src/test_trajectory_tracker_overshoot.cpp)
target_link_libraries(test_trajectory_tracker_overshoot Eigen3::Eigen rclcpp::rclcpp ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${tf2_geometry_msgs_TARGETS} ${trajectory_tracker_msgs_TARGETS})
add_launch_test(launch/trajectory_tracker_overshoot_test.launch.py TIMEOUT 120)

install(TARGETS
    test_trajectory_recorder
    test_trajectory_tracker
    test_trajectory_tracker_with_odom
    test_trajectory_tracker_overshoot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)