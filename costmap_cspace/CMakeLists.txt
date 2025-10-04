cmake_minimum_required(VERSION 3.14.4)
project(costmap_cspace)

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
  laser_geometry
  nav_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs

  costmap_cspace_msgs
)

find_package(ament_cmake REQUIRED)
foreach(COLCON_DEPEND ${COLCON_DEPENDS})
  find_package(${COLCON_DEPEND} REQUIRED)
endforeach()

add_library(costmap_3d_layers src/costmap_3d_layers.cpp)
target_include_directories(costmap_3d_layers PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(costmap_3d_layers ${COLCON_DEPENDS})

add_executable(costmap_3d src/costmap_3d.cpp)
target_include_directories(costmap_3d PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(costmap_3d costmap_3d_layers)
ament_target_dependencies(costmap_3d ${COLCON_DEPENDS})

add_executable(laserscan_to_map src/laserscan_to_map.cpp)
target_include_directories(laserscan_to_map PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(laserscan_to_map ${COLCON_DEPENDS})

add_executable(pointcloud2_to_map src/pointcloud2_to_map.cpp)
target_include_directories(pointcloud2_to_map PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(pointcloud2_to_map ${COLCON_DEPENDS})

add_executable(largemap_to_map src/largemap_to_map.cpp)
target_include_directories(largemap_to_map PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(largemap_to_map ${COLCON_DEPENDS})


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
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS costmap_3d_layers
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include
)

ament_package()
