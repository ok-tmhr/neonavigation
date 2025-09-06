cmake_minimum_required(VERSION 3.14.4)
project(trajectory_tracker)

set(COLCON_DEPENDS
  rclcpp

  dynamic_reconfigure
  geometry_msgs
  interactive_markers
  nav_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros

  trajectory_tracker_msgs
)

find_package(ament_cmake REQUIRED ${COLCON_DEPENDS})
foreach(COLCON_DEPEND ${COLCON_DEPENDS})
  find_package(${COLCON_DEPEND} REQUIRED)
endforeach()
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/TrajectoryTracker.cfg
)

catkin_package(
  INCLUDE_DIRS include
  COLCON_DEPENDS ${COLCON_DEPENDS}
)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(trajectory_tracker src/trajectory_tracker.cpp)
target_link_libraries(trajectory_tracker ${catkin_LIBRARIES})
add_dependencies(trajectory_tracker ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_recorder src/trajectory_recorder.cpp)
target_link_libraries(trajectory_recorder ${catkin_LIBRARIES})
add_dependencies(trajectory_recorder ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_saver src/trajectory_saver.cpp)
target_link_libraries(trajectory_saver ${catkin_LIBRARIES})
add_dependencies(trajectory_saver ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_server src/trajectory_server.cpp)
target_link_libraries(trajectory_server ${catkin_LIBRARIES})
add_dependencies(trajectory_server ${catkin_EXPORTED_TARGETS})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
    trajectory_recorder
    trajectory_saver
    trajectory_server
    trajectory_tracker
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include
)

ament_package()

