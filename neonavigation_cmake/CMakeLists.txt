cmake_minimum_required(VERSION 3.14.4)
project(neonavigation_cmake NONE)

find_package(ament_cmake_ros REQUIRED)

ament_package(CONFIG_EXTRAS neonavigation_common-extra.cmake)

install(DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
