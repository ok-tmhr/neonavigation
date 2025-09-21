include_directories(include)

find_package(Boost REQUIRED COMPONENTS thread chrono)

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
target_include_directories(planner_3d_debug PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(planner_3d_debug ${COLCON_DEPENDS})

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
target_link_libraries(test_distance_map_fast_update ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})
ament_target_dependencies(test_distance_map_fast_update ${COLCON_DEPENDS})

ament_add_gtest(test_grid_metric_converter src/test_grid_metric_converter.cpp)
target_link_libraries(test_grid_metric_converter ${Boost_LIBRARIES})
ament_target_dependencies(test_grid_metric_converter ${COLCON_DEPENDS})

ament_add_gtest(test_planner_3d_cost
  src/test_planner_3d_cost.cpp
  ../src/grid_astar_model_3dof.cpp
  ../src/motion_cache.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_planner_3d_cost ${Boost_LIBRARIES})
ament_target_dependencies(test_planner_3d_cost ${COLCON_DEPENDS})

ament_add_gtest(test_costmap_bbf
  src/test_costmap_bbf.cpp
  ../src/costmap_bbf.cpp
)
target_link_libraries(test_costmap_bbf ${Boost_LIBRARIES})

ament_add_gtest(test_motion_cache
  src/test_motion_cache.cpp
  ../src/motion_cache.cpp
)
target_link_libraries(test_motion_cache ${Boost_LIBRARIES})
ament_target_dependencies(test_motion_cache ${COLCON_DEPENDS})

ament_add_gtest(test_motion_primitive_builder
  src/test_motion_primitive_builder.cpp
  ../src/motion_primitive_builder.cpp
  ../src/rotation_cache.cpp
)
target_link_libraries(test_motion_primitive_builder ${Boost_LIBRARIES})
ament_target_dependencies(test_motion_primitive_builder ${COLCON_DEPENDS})

ament_add_gtest(test_start_pose_predictor
  src/test_start_pose_predictor.cpp
  ../src/motion_cache.cpp
  ../src/rotation_cache.cpp
  ../src/start_pose_predictor.cpp
)
target_link_libraries(test_start_pose_predictor ${Boost_LIBRARIES})
ament_target_dependencies(test_start_pose_predictor ${COLCON_DEPENDS})

ament_add_gtest(test_debug_outputs
  test/debug_outputs_rostest.test
  src/test_debug_outputs.cpp
)
target_link_libraries(test_debug_outputs)
ament_target_dependencies(test_debug_outputs ${COLCON_DEPENDS})

ament_add_gtest(test_navigate
  test/navigation_rostest.test
  src/test_navigate.cpp
)
target_link_libraries(test_navigate)
ament_target_dependencies(test_navigate ${COLCON_DEPENDS})

# # add_rostest(test/navigation_rostest.test
# #   ARGS antialias_start:=true
# #   DEPENDENCIES test_navigate
# # )

# # add_rostest(test/navigation_rostest.test
# #   ARGS antialias_start:=true fast_map_update:=true
# #   DEPENDENCIES test_navigate
# # )

# # add_rostest(test/navigation_rostest.test
# #   ARGS with_tolerance:=true
# #   DEPENDENCIES test_navigate
# # )

# # add_rostest(test/navigation_rostest.test
# #   ARGS enable_crowd_mode:=true
# #   DEPENDENCIES test_navigate
# # )

ament_add_gtest(test_navigate_remember
  test/navigation_remember_rostest.test
  src/test_navigate_remember.cpp
)
target_link_libraries(test_navigate_remember)
ament_target_dependencies(test_navigate_remember ${COLCON_DEPENDS})

# # add_rostest(test/navigation_compat_rostest.test
# #   DEPENDENCIES test_navigate
# # )

ament_add_gtest(test_navigate_boundary
  test/navigation_boundary_rostest.test
  src/test_navigate_boundary.cpp
)
target_link_libraries(test_navigate_boundary)
ament_target_dependencies(test_navigate_boundary ${COLCON_DEPENDS})

ament_add_gtest(test_abort
  test/abort_rostest.test
  src/test_abort.cpp
)
target_link_libraries(test_abort)
ament_target_dependencies(test_abort ${COLCON_DEPENDS})

ament_add_gtest(test_preempt
  test/preempt_rostest.test
  src/test_preempt.cpp
)
target_link_libraries(test_preempt)
ament_target_dependencies(test_preempt ${COLCON_DEPENDS})

ament_add_gtest(test_costmap_watchdog
  test/costmap_watchdog_rostest.test
  src/test_costmap_watchdog.cpp
)
target_link_libraries(test_costmap_watchdog)
ament_target_dependencies(test_costmap_watchdog ${COLCON_DEPENDS})

ament_add_gtest(test_planner_2dof_serial_joints
  test/planner_2dof_serial_joints_rostest.test
  src/test_planner_2dof_serial_joints.cpp
)
target_link_libraries(test_planner_2dof_serial_joints)
ament_target_dependencies(test_planner_2dof_serial_joints ${COLCON_DEPENDS})

ament_add_gtest(test_planner_3d_map_size
  test/planner_3d_map_size_rostest.test
  src/test_planner_3d_map_size.cpp
)
target_link_libraries(test_planner_3d_map_size)
ament_target_dependencies(test_planner_3d_map_size ${COLCON_DEPENDS})

ament_add_gtest(test_tolerant_action
  test/tolerant_action_rostest.test
  src/test_tolerant_action.cpp
)
target_link_libraries(test_tolerant_action)
ament_target_dependencies(test_tolerant_action ${COLCON_DEPENDS})

ament_add_gtest(test_dynamic_parameter_change
  test/dynamic_parameter_change_rostest.test
  src/test_dynamic_parameter_change.cpp
)
target_link_libraries(test_dynamic_parameter_change)
ament_target_dependencies(test_dynamic_parameter_change ${COLCON_DEPENDS})

if(NEONAVIGATION_EXTRA_TESTS)

  ament_add_gtest(test_blockmem_gridmap_performance
    src/test_blockmem_gridmap_performance.cpp
  )
  target_link_libraries(test_blockmem_gridmap_performance ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})
  # Force release build for performance test.
  set_target_properties(test_blockmem_gridmap_performance PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}")

  install(TARGETS test_blockmem_gridmap_performance
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(TARGETS
    planner_3d_debug
    test_blockmem_gridmap
    test_grid_astar
    test_cyclic_vec
    test_distance_map
    test_distance_map_fast_update
    test_grid_metric_converter
    test_planner_3d_cost
    test_costmap_bbf
    test_motion_cache
    test_motion_primitive_builder
    test_start_pose_predictor
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
    test_blockmem_gridmap_performance
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY data script test
  DESTINATION share/${PROJECT_NAME}/test
)