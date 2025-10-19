# planner_3d node with assertion
ament_auto_add_executable(planner_3d_debug
  ../src/costmap_bbf.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/planner_3d.cpp
  ../src/distance_map.cpp
  ../src/rotation_cache.cpp
  ../src/start_pose_predictor.cpp
)
target_compile_definitions(planner_3d_debug PUBLIC DEBUG)
target_link_libraries(planner_3d_debug ${OpenMP_CXX_FLAGS})
target_include_directories(planner_3d_debug PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_auto_add_gtest(test_blockmem_gridmap src/test_blockmem_gridmap.cpp)
target_include_directories(test_blockmem_gridmap PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_auto_add_gtest(test_grid_astar src/test_grid_astar.cpp)
target_link_libraries(test_grid_astar ${OpenMP_CXX_FLAGS})
target_include_directories(test_grid_astar PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_auto_add_gtest(test_cyclic_vec src/test_cyclic_vec.cpp)
target_include_directories(test_cyclic_vec PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_auto_add_gtest(test_distance_map
  src/test_distance_map.cpp
  ../src/costmap_bbf.cpp
  ../src/distance_map.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
  TIMEOUT 120
)
target_link_libraries(test_distance_map ${OpenMP_CXX_FLAGS})
target_include_directories(test_distance_map PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_auto_add_gtest(test_distance_map_fast_update
  src/test_distance_map_fast_update.cpp
  ../src/costmap_bbf.cpp
  ../src/distance_map.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_distance_map_fast_update ${OpenMP_CXX_FLAGS})
target_include_directories(test_distance_map_fast_update PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_auto_add_gtest(test_grid_metric_converter src/test_grid_metric_converter.cpp)
target_include_directories(test_grid_metric_converter PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_auto_add_gtest(test_planner_3d_cost
  src/test_planner_3d_cost.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_include_directories(test_planner_3d_cost PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_auto_add_gtest(test_costmap_bbf
  src/test_costmap_bbf.cpp
  ../src/costmap_bbf.cpp
)
target_include_directories(test_costmap_bbf PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_auto_add_gtest(test_motion_cache
  src/test_motion_cache.cpp
  ../src/motion_cache.cpp
)
target_include_directories(test_motion_cache PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_auto_add_gtest(test_motion_primitive_builder
  src/test_motion_primitive_builder.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_include_directories(test_motion_primitive_builder PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_auto_add_gtest(test_start_pose_predictor
  src/test_start_pose_predictor.cpp
  ../src/motion_cache.cpp
  ../src/rotation_cache.cpp
  ../src/start_pose_predictor.cpp
)
target_include_directories(test_start_pose_predictor PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_auto_add_gtest(test_debug_outputs SKIP_TEST
  src/test_debug_outputs.cpp
)
add_launch_test(launch/debug_outputs_test.launch.py TIMEOUT 15)

ament_auto_add_gtest(test_navigate SKIP_TEST
  src/test_navigate.cpp
)

add_launch_test(launch/navigation_test.launch.py
  TARGET test_navigation_rostest0
  TIMEOUT 200
)

add_launch_test(launch/navigation_test.launch.py
  ARGS antialias_start:=true
  TARGET test_navigation_rostest1
  TIMEOUT 200
)

add_launch_test(launch/navigation_test.launch.py
  ARGS antialias_start:=true fast_map_update:=true
  TARGET test_navigation_rostest2
  TIMEOUT 200
)

add_launch_test(launch/navigation_test.launch.py
  ARGS with_tolerance:=true
  TARGET test_navigation_rostest3
  TIMEOUT 200
)

add_launch_test(launch/navigation_test.launch.py
  ARGS enable_crowd_mode:=true
  TARGET test_navigation_rostest4
  TIMEOUT 200
)

ament_auto_add_gtest(test_navigate_remember SKIP_TEST
  src/test_navigate_remember.cpp
)
add_launch_test(launch/navigation_remember_test.launch.py TIMEOUT 200)

ament_auto_add_gtest(test_navigate_boundary SKIP_TEST
  src/test_navigate_boundary.cpp
)
add_launch_test(launch/navigation_boundary_test.launch.py TIMEOUT 45)

ament_auto_add_gtest(test_abort SKIP_TEST
  src/test_abort.cpp
)
add_launch_test(launch/abort_test.launch.py TIMEOUT 90)

ament_auto_add_gtest(test_preempt SKIP_TEST
  src/test_preempt.cpp
)
add_launch_test(launch/preempt_test.launch.py TIMEOUT 90)

ament_auto_add_gtest(test_costmap_watchdog SKIP_TEST
  src/test_costmap_watchdog.cpp
)
add_launch_test(launch/costmap_watchdog_test.launch.py TIMEOUT 30)

ament_auto_add_gtest(test_planner_2dof_serial_joints SKIP_TEST
  src/test_planner_2dof_serial_joints.cpp
)
add_launch_test(launch/planner_2dof_serial_joints_test.launch.py TIMEOUT 60)

ament_auto_add_gtest(test_planner_3d_map_size SKIP_TEST
  src/test_planner_3d_map_size.cpp
)
add_launch_test(launch/planner_3d_map_size_test.launch.py)

ament_auto_add_gtest(test_tolerant_action SKIP_TEST
  src/test_tolerant_action.cpp
)
add_launch_test(launch/tolerant_action_test.launch.py TIMEOUT 90)

ament_auto_add_gtest(test_dynamic_parameter_change SKIP_TEST
  src/test_dynamic_parameter_change.cpp
)
add_launch_test(launch/dynamic_parameter_change_test.launch.py TIMEOUT 90)

if(NEONAVIGATION_EXTRA_TESTS)

  ament_auto_add_gtest(test_blockmem_gridmap_performance
    src/test_blockmem_gridmap_performance.cpp
  )
  target_link_libraries(test_blockmem_gridmap_performance ${OpenMP_CXX_FLAGS})
  # Force release build for performance test.
  set_target_properties(test_blockmem_gridmap_performance PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}")
  target_include_directories(test_blockmem_gridmap_performance PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

endif()

install(DIRECTORY test data script
  DESTINATION share/${PROJECT_NAME}
)
