cmake_minimum_required(VERSION 3.14.4)
project(neonavigation_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

