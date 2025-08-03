include_directories(include)

ament_add_gtest(test_${PROJECT_NAME}_filter src/test_filter.cpp)
target_include_directories(test_${PROJECT_NAME}_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>)

ament_add_gtest(test_${PROJECT_NAME}_path2d src/test_path2d.cpp)
target_include_directories(test_${PROJECT_NAME}_path2d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(test_${PROJECT_NAME}_path2d ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_trajectory_recorder src/test_trajectory_recorder.cpp)
# add_rostest_gtest(test_trajectory_recorder test/trajectory_recorder_rostest.test
#     src/test_trajectory_recorder.cpp)
# target_link_libraries(test_trajectory_recorder ${catkin_LIBRARIES})
ament_target_dependencies(test_trajectory_recorder ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_trajectory_tracker src/test_trajectory_tracker.cpp)
ament_target_dependencies(test_trajectory_tracker ${AMENT_CMAKE_DEPENDS} Eigen3)
# add_launch_test(test/test_trajectory_tracker.launch.py ${ARGN})

# target_link_libraries(test_trajectory_tracker ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(test_trajectory_tracker trajectory_tracker)

# add_rostest(test/trajectory_tracker_rostest.test
#   ARGS odom_delay:=0.04 use_odom:=true use_time_optimal_control:=true
#   DEPENDENCIES test_trajectory_tracker
# )

# add_rostest(test/trajectory_tracker_rostest.test
#   ARGS odom_delay:=0.04 use_odom:=true use_time_optimal_control:=false
#   DEPENDENCIES test_trajectory_tracker
# )

# add_rostest_gtest(test_trajectory_tracker_with_odom test/trajectory_tracker_with_odom_rostest.test
#     src/test_trajectory_tracker_with_odom.cpp)
# target_link_libraries(test_trajectory_tracker_with_odom ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(test_trajectory_tracker_with_odom trajectory_tracker)

# ament_add_gtest(test_trajectory_tracker_overshoot src/test_trajectory_tracker_overshoot.cpp)
# add_rostest_gtest(test_trajectory_tracker_overshoot test/trajectory_tracker_overshoot_rostest.test
#     src/test_trajectory_tracker_overshoot.cpp)
# target_link_libraries(test_trajectory_tracker_overshoot ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(test_trajectory_tracker_overshoot trajectory_tracker)
# ament_target_dependencies(test_trajectory_tracker_overshoot ${AMENT_CMAKE_DEPENDS})
