cmake_minimum_required(VERSION 3.14.4)
project(costmap_cspace)

set(COLCON_DEPENDS
  rclcpp

  geometry_msgs
  laser_geometry
  nav_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs

  costmap_cspace_msgs
)

find_package(ament_cmake REQUIRED ${COLCON_DEPENDS})
foreach(COLCON_DEPEND ${COLCON_DEPENDS})
  find_package(${COLCON_DEPEND} REQUIRED)
endforeach()
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES costmap_3d_layers
  COLCON_DEPENDS ${COLCON_DEPENDS}
)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/costmap_cspace)

add_library(costmap_3d_layers src/costmap_3d_layers.cpp)
target_link_libraries(costmap_3d_layers ${catkin_LIBRARIES})
add_dependencies(costmap_3d_layers ${catkin_EXPORTED_TARGETS})

add_executable(costmap_3d src/costmap_3d.cpp)
target_link_libraries(costmap_3d costmap_3d_layers ${catkin_LIBRARIES})
add_dependencies(costmap_3d ${catkin_EXPORTED_TARGETS})

add_executable(laserscan_to_map src/laserscan_to_map.cpp)
target_link_libraries(laserscan_to_map ${catkin_LIBRARIES})
add_dependencies(laserscan_to_map ${catkin_EXPORTED_TARGETS})

add_executable(pointcloud2_to_map src/pointcloud2_to_map.cpp)
target_link_libraries(pointcloud2_to_map ${catkin_LIBRARIES})
add_dependencies(pointcloud2_to_map ${catkin_EXPORTED_TARGETS})

add_executable(largemap_to_map src/largemap_to_map.cpp)
target_link_libraries(largemap_to_map ${catkin_LIBRARIES})
add_dependencies(largemap_to_map ${catkin_EXPORTED_TARGETS})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
    costmap_3d
    largemap_to_map
    laserscan_to_map
    pointcloud2_to_map
    costmap_3d_layers
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include
)

ament_package()
