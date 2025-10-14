cmake_minimum_required(VERSION 3.14.4)
project(costmap_cspace)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(costmap_3d_layers src/costmap_3d_layers.cpp)

ament_auto_add_executable(costmap_3d src/costmap_3d.cpp)
target_link_libraries(costmap_3d costmap_3d_layers)

ament_auto_add_executable(laserscan_to_map src/laserscan_to_map.cpp)

ament_auto_add_executable(pointcloud2_to_map src/pointcloud2_to_map.cpp)

ament_auto_add_executable(largemap_to_map src/largemap_to_map.cpp)


if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
