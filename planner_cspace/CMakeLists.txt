cmake_minimum_required(VERSION 3.14.4)
project(planner_cspace)

set(AMENT_CMAKE_DEPENDS
  roscpp

  geometry_msgs
  move_base_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs

  costmap_cspace
  costmap_cspace_msgs
  neonavigation_metrics_msgs
  planner_cspace_msgs
  trajectory_tracker
  trajectory_tracker_msgs
)

find_package(ament_cmake REQUIRED)
foreach(DEPEND ${AMENT_CMAKE_DEPENDS})
  find_package(DEPEND REQUIRED)
endforeach()
find_package(OpenMP REQUIRED)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-funroll-loops -finline-functions ${OpenMP_CXX_FLAGS})


add_executable(planner_3d
  src/costmap_bbf.cpp
  src/grid_astar_model_3dof.cpp
  src/motion_cache.cpp
  src/motion_primitive_builder.cpp
  src/planner_3d.cpp
  src/distance_map.cpp
  src/rotation_cache.cpp
  src/start_pose_predictor.cpp
)
# target_link_libraries(planner_3d ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})
ament_export_dependencies(planner_3d ${AMENT_CMAKE_DEPENDS} OpenMP)
set_property(TARGET planner_3d PROPERTY PUBLIC_HEADER
  include/planner_cspace/bbf.h
  include/planner_cspace/blockmem_gridmap.h
  include/planner_cspace/cyclic_vec.h
  include/planner_cspace/grid_astar.h
  include/planner_cspace/grid_astar_model.h
  include/planner_cspace/jump_detector.h
  include/planner_cspace/reservable_priority_queue.h
)

add_executable(planner_2dof_serial_joints
  src/planner_2dof_serial_joints.cpp
  src/grid_astar_model_2dof_joints.cpp
)
target_link_libraries(planner_2dof_serial_joints ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_CXX_FLAGS})
add_dependencies(planner_2dof_serial_joints ${catkin_EXPORTED_TARGETS})

add_executable(dummy_robot src/dummy_robot.cpp)
target_link_libraries(dummy_robot ${catkin_LIBRARIES})
add_dependencies(dummy_robot ${catkin_EXPORTED_TARGETS})

add_executable(patrol src/patrol.cpp)
target_link_libraries(patrol ${catkin_LIBRARIES})
add_dependencies(patrol ${catkin_EXPORTED_TARGETS})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()


install(TARGETS
    dummy_robot
    patrol
    planner_2dof_serial_joints
    planner_3d
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PUBLIC_HEADER DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
