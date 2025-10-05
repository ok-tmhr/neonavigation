cmake_minimum_required(VERSION 3.14.4)
project(trajectory_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wshadow)
endif()

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

find_package(ament_cmake REQUIRED)
foreach(COLCON_DEPEND ${COLCON_DEPENDS})
  find_package(${COLCON_DEPEND} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)


include_directories(include)


add_executable(trajectory_tracker src/trajectory_tracker.cpp)
target_include_directories(trajectory_tracker PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(trajectory_tracker Eigen3 ${COLCON_DEPENDS})

add_executable(trajectory_recorder src/trajectory_recorder.cpp)
target_include_directories(trajectory_recorder PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(trajectory_recorder ${COLCON_DEPENDS})

add_executable(trajectory_saver src/trajectory_saver.cpp)
target_include_directories(trajectory_saver PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(trajectory_saver ${COLCON_DEPENDS})

add_executable(trajectory_server src/trajectory_server.cpp)
target_include_directories(trajectory_server PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(trajectory_server ${COLCON_DEPENDS})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
    trajectory_recorder
    trajectory_saver
    trajectory_server
    trajectory_tracker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()

