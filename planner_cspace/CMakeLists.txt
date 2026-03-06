cmake_minimum_required(VERSION 3.14.4)
project(planner_cspace)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenMP REQUIRED)

add_compile_options(-funroll-loops -finline-functions ${OpenMP_CXX_FLAGS})

generate_parameter_library(
  planner_2dof_serial_joints_parameters
  param/planner_2dof_serial_joints_parameters.yaml
)

ament_auto_add_executable(planner_3d
  src/costmap_bbf.cpp
  src/grid_astar_model_3dof.cpp
  src/motion_cache.cpp
  src/motion_primitive_builder.cpp
  src/planner_3d.cpp
  src/distance_map.cpp
  src/rotation_cache.cpp
  src/start_pose_predictor.cpp
)
target_link_libraries(planner_3d ${OpenMP_CXX_FLAGS})
set_property(TARGET planner_3d PROPERTY PUBLIC_HEADER
  include/planner_cspace/bbf.h
  include/planner_cspace/blockmem_gridmap.h
  include/planner_cspace/cyclic_vec.h
  include/planner_cspace/grid_astar.h
  include/planner_cspace/grid_astar_model.h
  include/planner_cspace/jump_detector.h
  include/planner_cspace/reservable_priority_queue.h
)

ament_auto_add_executable(planner_2dof_serial_joints
  src/planner_2dof_serial_joints.cpp
  src/grid_astar_model_2dof_joints.cpp
)
target_link_libraries(planner_2dof_serial_joints ${OpenMP_CXX_FLAGS} planner_2dof_serial_joints_parameters)

ament_auto_add_executable(dummy_robot src/dummy_robot.cpp)

ament_auto_add_executable(patrol src/patrol.cpp)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

