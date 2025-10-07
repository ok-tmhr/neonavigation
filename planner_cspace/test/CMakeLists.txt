include_directories(include)

# planner_3d node with assertion
add_executable(planner_3d_debug
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
ament_target_dependencies(planner_3d_debug ${COLCON_DEPENDS})

ament_add_gtest(test_blockmem_gridmap src/test_blockmem_gridmap.cpp)
target_link_libraries(test_blockmem_gridmap)

ament_add_gtest(test_grid_astar src/test_grid_astar.cpp)
target_link_libraries(test_grid_astar ${OpenMP_CXX_FLAGS})

ament_add_gtest(test_cyclic_vec src/test_cyclic_vec.cpp)
target_link_libraries(test_cyclic_vec)

ament_add_gtest(test_distance_map
  src/test_distance_map.cpp
  ../src/costmap_bbf.cpp
  ../src/distance_map.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_distance_map ${OpenMP_CXX_FLAGS})
ament_target_dependencies(test_distance_map ${COLCON_DEPENDS})

ament_add_gtest(test_distance_map_fast_update
  src/test_distance_map_fast_update.cpp
  ../src/costmap_bbf.cpp
  ../src/distance_map.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_distance_map_fast_update ${OpenMP_CXX_FLAGS})
ament_target_dependencies(test_distance_map_fast_update ${COLCON_DEPENDS})

ament_add_gtest(test_grid_metric_converter src/test_grid_metric_converter.cpp)
target_link_libraries(test_grid_metric_converter)
ament_target_dependencies(test_grid_metric_converter ${COLCON_DEPENDS})

ament_add_gtest(test_planner_3d_cost
  src/test_planner_3d_cost.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_planner_3d_cost)
ament_target_dependencies(test_planner_3d_cost ${COLCON_DEPENDS})

ament_add_gtest(test_costmap_bbf
  src/test_costmap_bbf.cpp
  ../src/costmap_bbf.cpp
)
target_link_libraries(test_costmap_bbf)

ament_add_gtest(test_motion_cache
  src/test_motion_cache.cpp
  ../src/motion_cache.cpp
)
target_link_libraries(test_motion_cache)
ament_target_dependencies(test_motion_cache ${COLCON_DEPENDS})

ament_add_gtest(test_motion_primitive_builder
  src/test_motion_primitive_builder.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_motion_primitive_builder)
ament_target_dependencies(test_motion_primitive_builder ${COLCON_DEPENDS})

ament_add_gtest(test_start_pose_predictor
  src/test_start_pose_predictor.cpp
  ../src/motion_cache.cpp
  ../src/rotation_cache.cpp
  ../src/start_pose_predictor.cpp
)
target_link_libraries(test_start_pose_predictor)
ament_target_dependencies(test_start_pose_predictor ${COLCON_DEPENDS})

ament_add_gtest_executable(test_debug_outputs
  test/debug_outputs_rostest.test
  src/test_debug_outputs.cpp
)
target_link_libraries(test_debug_outputs)
ament_target_dependencies(test_debug_outputs ${COLCON_DEPENDS})
add_launch_test(launch/debug_outputs_test.launch.py TIMEOUT 15)

ament_add_gtest_executable(test_navigate
  src/test_navigate.cpp
)
target_link_libraries(test_navigate)
ament_target_dependencies(test_navigate ${COLCON_DEPENDS})

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

ament_add_gtest_executable(test_navigate_remember
  src/test_navigate_remember.cpp
)
target_link_libraries(test_navigate_remember)
ament_target_dependencies(test_navigate_remember ${COLCON_DEPENDS})
add_launch_test(launch/navigation_remember_test.launch.py TIMEOUT 200)

ament_add_gtest_executable(test_navigate_boundary
  src/test_navigate_boundary.cpp
)
target_link_libraries(test_navigate_boundary)
ament_target_dependencies(test_navigate_boundary ${COLCON_DEPENDS})
add_launch_test(launch/navigation_boundary_test.launch.py TIMEOUT 45)

ament_add_gtest_executable(test_abort
  src/test_abort.cpp
)
target_link_libraries(test_abort)
ament_target_dependencies(test_abort ${COLCON_DEPENDS})
add_launch_test(launch/abort_test.launch.py TIMEOUT 90)

ament_add_gtest_executable(test_preempt
  src/test_preempt.cpp
)
target_link_libraries(test_preempt)
ament_target_dependencies(test_preempt ${COLCON_DEPENDS})
add_launch_test(launch/preempt_test.launch.py TIMEOUT 90)

ament_add_gtest_executable(test_costmap_watchdog
  src/test_costmap_watchdog.cpp
)
target_link_libraries(test_costmap_watchdog)
ament_target_dependencies(test_costmap_watchdog ${COLCON_DEPENDS})
add_launch_test(launch/costmap_watchdog_test.launch.py TIMEOUT 30)

ament_add_gtest_executable(test_planner_2dof_serial_joints
  src/test_planner_2dof_serial_joints.cpp
)
target_link_libraries(test_planner_2dof_serial_joints)
ament_target_dependencies(test_planner_2dof_serial_joints ${COLCON_DEPENDS})
add_launch_test(launch/planner_2dof_serial_joints_test.launch.py TIMEOUT 60)

ament_add_gtest_executable(test_planner_3d_map_size
  src/test_planner_3d_map_size.cpp
)
target_link_libraries(test_planner_3d_map_size)
ament_target_dependencies(test_planner_3d_map_size ${COLCON_DEPENDS})
add_launch_test(launch/planner_3d_map_size_test.launch.py)

ament_add_gtest_executable(test_tolerant_action
  src/test_tolerant_action.cpp
)
target_link_libraries(test_tolerant_action)
ament_target_dependencies(test_tolerant_action ${COLCON_DEPENDS})
add_launch_test(launch/tolerant_action_test.launch.py TIMEOUT 90)

ament_add_gtest_executable(test_dynamic_parameter_change
  src/test_dynamic_parameter_change.cpp
)
target_link_libraries(test_dynamic_parameter_change)
ament_target_dependencies(test_dynamic_parameter_change ${COLCON_DEPENDS})
add_launch_test(launch/dynamic_parameter_change_test.launch.py TIMEOUT 90)

if(NEONAVIGATION_EXTRA_TESTS)

  ament_add_gtest(test_blockmem_gridmap_performance
    src/test_blockmem_gridmap_performance.cpp
  )
  target_link_libraries(test_blockmem_gridmap_performance ${OpenMP_CXX_FLAGS})
  # Force release build for performance test.
  set_target_properties(test_blockmem_gridmap_performance PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}")

endif()

install(TARGETS
    planner_3d_debug
    test_debug_outputs
    test_navigate
    test_navigate_remember
    test_navigate_boundary
    test_abort
    test_preempt
    test_costmap_watchdog
    test_planner_2dof_serial_joints
    test_planner_3d_map_size
    test_tolerant_action
    test_dynamic_parameter_change
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY data script
  DESTINATION share/${PROJECT_NAME}/test
)