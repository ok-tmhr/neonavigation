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
target_link_libraries(planner_3d_debug ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})
ament_target_dependencies(planner_3d_debug ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_blockmem_gridmap src/test_blockmem_gridmap.cpp)
target_link_libraries(test_blockmem_gridmap ${Boost_LIBRARIES})

ament_add_gtest(test_grid_astar src/test_grid_astar.cpp)
target_link_libraries(test_grid_astar ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})

ament_add_gtest(test_cyclic_vec src/test_cyclic_vec.cpp)
target_link_libraries(test_cyclic_vec ${Boost_LIBRARIES})

ament_add_gtest(test_distance_map
  src/test_distance_map.cpp
  ../src/costmap_bbf.cpp
  ../src/distance_map.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_distance_map ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})
ament_target_dependencies(test_distance_map ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_distance_map_fast_update
  src/test_distance_map_fast_update.cpp
  ../src/costmap_bbf.cpp
  ../src/distance_map.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_distance_map_fast_update ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})
ament_target_dependencies(test_distance_map_fast_update ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_grid_metric_converter src/test_grid_metric_converter.cpp)
target_link_libraries(test_grid_metric_converter ${Boost_LIBRARIES})
ament_target_dependencies(test_grid_metric_converter ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_planner_3d_cost
  src/test_planner_3d_cost.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_planner_3d_cost ${Boost_LIBRARIES})
ament_target_dependencies(test_planner_3d_cost ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_costmap_bbf
  src/test_costmap_bbf.cpp
  ../src/costmap_bbf.cpp
)
target_link_libraries(test_costmap_bbf ${Boost_LIBRARIES})
ament_target_dependencies(test_costmap_bbf ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_motion_cache
  src/test_motion_cache.cpp
  ../src/motion_cache.cpp
)
target_link_libraries(test_motion_cache ${Boost_LIBRARIES})
ament_target_dependencies(test_motion_cache ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_motion_primitive_builder
  src/test_motion_primitive_builder.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_motion_primitive_builder ${Boost_LIBRARIES})
ament_target_dependencies(test_motion_primitive_builder ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_start_pose_predictor
  src/test_start_pose_predictor.cpp
  ../src/motion_cache.cpp
  ../src/rotation_cache.cpp
  ../src/start_pose_predictor.cpp
)
target_link_libraries(test_start_pose_predictor ${Boost_LIBRARIES})
ament_target_dependencies(test_start_pose_predictor ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_debug_outputs
  src/test_debug_outputs.cpp
)
ament_target_dependencies(test_debug_outputs ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_navigate
  src/test_navigate.cpp
)
ament_target_dependencies(test_navigate ${AMENT_CMAKE_DEPENDS})

# add_rostest(test/navigation_rostest.test
#   ARGS antialias_start:=true
#   DEPENDENCIES test_navigate
# )

# add_rostest(test/navigation_rostest.test
#   ARGS antialias_start:=true fast_map_update:=true
#   DEPENDENCIES test_navigate
# )

# add_rostest(test/navigation_rostest.test
#   ARGS with_tolerance:=true
#   DEPENDENCIES test_navigate
# )

# add_rostest(test/navigation_rostest.test
#   ARGS enable_crowd_mode:=true
#   DEPENDENCIES test_navigate
# )

ament_add_gtest(test_navigate_remember
  src/test_navigate_remember.cpp
)
ament_target_dependencies(test_navigate_remember ${AMENT_CMAKE_DEPENDS})

# add_rostest(test/navigation_compat_rostest.test
#   DEPENDENCIES test_navigate
# )

ament_add_gtest(test_navigate_boundary
  src/test_navigate_boundary.cpp
)
ament_target_dependencies(test_navigate_boundary ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_abort
  test/abort_rostest.test
  src/test_abort.cpp
)
ament_target_dependencies(test_abort ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_preempt
  test/preempt_rostest.test
  src/test_preempt.cpp
)
ament_target_dependencies(test_preempt ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_costmap_watchdog
  test/costmap_watchdog_rostest.test
  src/test_costmap_watchdog.cpp
)
ament_target_dependencies(test_costmap_watchdog ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_planner_2dof_serial_joints
  test/planner_2dof_serial_joints_rostest.test
  src/test_planner_2dof_serial_joints.cpp
)
ament_target_dependencies(test_planner_2dof_serial_joints ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_planner_3d_map_size
  test/planner_3d_map_size_rostest.test
  src/test_planner_3d_map_size.cpp
)
ament_target_dependencies(test_planner_3d_map_size ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_tolerant_action
  test/tolerant_action_rostest.test
  src/test_tolerant_action.cpp
)
ament_target_dependencies(test_tolerant_action ${AMENT_CMAKE_DEPENDS})

ament_add_gtest(test_dynamic_parameter_change
  src/test_dynamic_parameter_change.cpp
)
ament_target_dependencies(test_dynamic_parameter_change ${AMENT_CMAKE_DEPENDS})

if(NEONAVIGATION_EXTRA_TESTS)

  ament_add_gtest(test_blockmem_gridmap_performance
    src/test_blockmem_gridmap_performance.cpp
  )
  target_link_libraries(test_blockmem_gridmap_performance ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})
  # Force release build for performance test.
  set_target_properties(test_blockmem_gridmap_performance PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}")

endif()

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}/test
)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}/test
)