cmake_minimum_required(VERSION 3.14.4)
project(neonavigation_launch)

find_package(ament_cmake REQUIRED)
catkin_package(
	AMENT_CMAKE_DEPENDS
)


install(DIRECTORY launch config map
  DESTINATION share/${PROJECT_NAME}
)
