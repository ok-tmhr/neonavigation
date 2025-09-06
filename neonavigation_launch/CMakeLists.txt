cmake_minimum_required(VERSION 3.14.4)
project(neonavigation_launch)

find_package(ament_cmake REQUIRED)
catkin_package(
	COLCON_DEPENDS
)


install(DIRECTORY launch config map
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

ament_package()

