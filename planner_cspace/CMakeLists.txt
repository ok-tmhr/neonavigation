cmake_minimum_required(VERSION 3.14.4)
project(planner_cspace)

find_package(neonavigation_cmake REQUIRED)
neonavigation_find_build_dependencies()
find_package(Eigen3 COMPONENTS Core)

find_package(OpenMP REQUIRED)

add_compile_options(-funroll-loops -finline-functions ${OpenMP_CXX_FLAGS})

neonavigation_add_executable(planner_3d
  HAS_HEADER
  src/costmap_bbf.cpp
  src/grid_astar_model_3dof.cpp
  src/motion_cache.cpp
  src/motion_primitive_builder.cpp
  src/planner_3d.cpp
  src/distance_map.cpp
  src/rotation_cache.cpp
  src/start_pose_predictor.cpp
  LIBRARIES
    ${OpenMP_CXX_FLAGS}
    rclcpp::rclcpp
    costmap_cspace_msgs
    planner_cspace_msgs
    tf2::tf2
    nav_msgs
    tf2_geometry_msgs
    neonavigation_metrics_msgs
    sensor_msgs
    std_srvs
    trajectory_tracker_msgs
    nav2_msgs
    diagnostic_updater::diagnostic_updater
    trajectory_tracker::path2d
    Eigen3::Eigen
)

neonavigation_add_executable(planner_2dof_serial_joints
  HAS_HEADER
  src/planner_2dof_serial_joints.cpp
  src/grid_astar_model_2dof_joints.cpp
  LIBRARIES
    ${OpenMP_CXX_FLAGS}
    rclcpp::rclcpp
    planner_cspace_msgs
    trajectory_msgs
    sensor_msgs
    tf2_ros::tf2_ros
  PARAMETER
    param/planner_2dof_serial_joints_parameters.yaml
)

neonavigation_add_executable(dummy_robot src/dummy_robot.cpp
   LIBRARIES
    rclcpp::rclcpp
    geometry_msgs
    nav_msgs
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros
)

neonavigation_add_executable(patrol src/patrol.cpp
   LIBRARIES
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    nav2_msgs
    planner_cspace_msgs
    nav_msgs
)

if(BUILD_TESTING)
  neonavigation_find_test_dependencies()
  # add_subdirectory(test)
endif()

neonavigation_package()

