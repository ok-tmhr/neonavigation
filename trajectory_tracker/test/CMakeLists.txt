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
ament_target_dependencies(test_trajectory_recorder ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_trajectory_tracker src/test_trajectory_tracker.cpp)
ament_target_dependencies(test_trajectory_tracker ${AMENT_CMAKE_DEPENDS} Eigen3)

# TODO Migrate rostest with *.test to Humble

# add_rostest(test/trajectory_tracker_rostest.test
#   ARGS odom_delay:=0.04 use_odom:=true use_time_optimal_control:=true
#   DEPENDENCIES test_trajectory_tracker
# )

# add_rostest(test/trajectory_tracker_rostest.test
#   ARGS odom_delay:=0.04 use_odom:=true use_time_optimal_control:=false
#   DEPENDENCIES test_trajectory_tracker
# )

ament_add_gtest(test_trajectory_tracker_with_odom src/test_trajectory_tracker_overshoot.cpp)
ament_target_dependencies(test_trajectory_tracker_with_odom ${AMENT_CMAKE_DEPENDS} Eigen3)

ament_add_gtest(test_trajectory_tracker_overshoot src/test_trajectory_tracker_overshoot.cpp)
target_include_directories(test_trajectory_tracker_overshoot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(test_trajectory_tracker_overshoot ${AMENT_CMAKE_DEPENDS})
