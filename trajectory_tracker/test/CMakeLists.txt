
ament_auto_add_gtest(test_${PROJECT_NAME}_filter src/test_filter.cpp)
target_include_directories(test_${PROJECT_NAME}_filter PRIVATE ${CMAKE_SOURCE_DIR}/include)

ament_auto_add_gtest(test_${PROJECT_NAME}_path2d src/test_path2d.cpp)
target_include_directories(test_${PROJECT_NAME}_path2d PRIVATE ${CMAKE_SOURCE_DIR}/include)

ament_auto_add_gtest(test_trajectory_recorder SKIP_TEST
    src/test_trajectory_recorder.cpp)
add_launch_test(launch/trajectory_recorder_test.launch.py TIMEOUT 60)

ament_auto_add_gtest(test_trajectory_tracker SKIP_TEST
    src/test_trajectory_tracker.cpp)

add_launch_test(launch/trajectory_tracker_test.launch.py
  TARGET test_trajectory_tracker_rostest0
  TIMEOUT 480 # TODO pass within 120
)

add_launch_test(launch/trajectory_tracker_test.launch.py
  ARGS odom_delay:=0.04 use_odom:=true use_time_optimal_control:=true
  TARGET test_trajectory_tracker_rostest1
  TIMEOUT 480 # TODO pass within 120
)

add_launch_test(launch/trajectory_tracker_test.launch.py
  ARGS odom_delay:=0.04 use_odom:=true use_time_optimal_control:=false
  TARGET test_trajectory_tracker_rostest2
  TIMEOUT 480 # TODO pass within 120
)

ament_auto_add_gtest(test_trajectory_tracker_with_odom SKIP_TEST
    src/test_trajectory_tracker_with_odom.cpp)
add_launch_test(launch/trajectory_tracker_with_odom_test.launch.py TIMEOUT 120)

ament_auto_add_gtest(test_trajectory_tracker_overshoot SKIP_TEST
    src/test_trajectory_tracker_overshoot.cpp)
add_launch_test(launch/trajectory_tracker_overshoot_test.launch.py TIMEOUT 120)


install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)