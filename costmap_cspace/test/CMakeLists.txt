
ament_auto_add_gtest(test_costmap_3d src/test_costmap_3d.cpp)
target_include_directories(test_costmap_3d PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_auto_add_gtest(test_pointcloud_accumulator src/test_pointcloud_accumulator.cpp)
target_include_directories(test_pointcloud_accumulator PRIVATE
   "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
