cmake_minimum_required(VERSION 3.14.4)
project(costmap_cspace)

set(COLCON_DEPENDS
  geometry_msgs
  laser_geometry
  nav_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs

  costmap_cspace_msgs
  # neonavigation_common
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
foreach(PKG ${COLCON_DEPENDS})
  find_package(${PKG} REQUIRED)
endforeach()


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_library(costmap_3d_layers src/costmap_3d_layers.cpp)
target_link_libraries(costmap_3d_layers rclcpp::rclcpp)
foreach(PKG ${COLCON_DEPENDS})
  target_link_libraries(costmap_3d_layers ${${PKG}_TARGETS})
endforeach()
target_include_directories(costmap_3d_layers PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(costmap_3d_layers rclcpp ${COLCON_DEPENDS})

add_executable(costmap_3d src/costmap_3d.cpp)
target_link_libraries(costmap_3d costmap_3d_layers rclcpp::rclcpp)
foreach(PKG ${COLCON_DEPENDS})
  target_link_libraries(costmap_3d ${${PKG}_TARGETS})
endforeach()
target_include_directories(costmap_3d PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(costmap_3d rclcpp ${COLCON_DEPENDS})

add_executable(laserscan_to_map src/laserscan_to_map.cpp)
target_link_libraries(laserscan_to_map rclcpp::rclcpp)
foreach(PKG ${COLCON_DEPENDS})
  target_link_libraries(laserscan_to_map ${${PKG}_TARGETS})
endforeach()
target_include_directories(laserscan_to_map PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(laserscan_to_map rclcpp ${COLCON_DEPENDS})

add_executable(pointcloud2_to_map src/pointcloud2_to_map.cpp)
target_link_libraries(pointcloud2_to_map rclcpp::rclcpp)
foreach(PKG ${COLCON_DEPENDS})
  target_link_libraries(pointcloud2_to_map ${${PKG}_TARGETS})
endforeach()
target_include_directories(pointcloud2_to_map PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(pointcloud2_to_map rclcpp ${COLCON_DEPENDS})

add_executable(largemap_to_map src/largemap_to_map.cpp)
target_link_libraries(largemap_to_map rclcpp::rclcpp)
foreach(PKG ${COLCON_DEPENDS})
  target_link_libraries(largemap_to_map ${${PKG}_TARGETS})
endforeach()
target_include_directories(largemap_to_map PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(largemap_to_map rclcpp ${COLCON_DEPENDS})


# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   add_subdirectory(test)

#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()


install(TARGETS
    costmap_3d
    largemap_to_map
    laserscan_to_map
    pointcloud2_to_map
    costmap_3d_layers
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()