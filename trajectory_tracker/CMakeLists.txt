cmake_minimum_required(VERSION 3.14.4)
project(trajectory_tracker)

set(COLCON_DEPENDS
  rclcpp

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


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include ${Boost_INCLUDE_DIRS})


add_executable(trajectory_tracker src/trajectory_tracker.cpp)
target_include_directories(trajectory_tracker PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(trajectory_tracker)
add_dependencies(trajectory_tracker)

add_executable(trajectory_recorder src/trajectory_recorder.cpp)
target_include_directories(trajectory_recorder PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(trajectory_recorder)
add_dependencies(trajectory_recorder)

add_executable(trajectory_saver src/trajectory_saver.cpp)
target_include_directories(trajectory_saver PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(trajectory_saver)
add_dependencies(trajectory_saver)

add_executable(trajectory_server src/trajectory_server.cpp)
target_include_directories(trajectory_server PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(trajectory_server)
add_dependencies(trajectory_server)


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

