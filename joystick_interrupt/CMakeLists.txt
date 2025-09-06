cmake_minimum_required(VERSION 3.14.4)
project(joystick_interrupt)

set(COLCON_DEPENDS
  rclcpp

  geometry_msgs
  sensor_msgs
  topic_tools

)

find_package(ament_cmake REQUIRED ${COLCON_DEPENDS})
foreach(COLCON_DEPEND ${COLCON_DEPENDS})
  find_package(${COLCON_DEPEND} REQUIRED)
endforeach()
catkin_package(
  COLCON_DEPENDS ${COLCON_DEPENDS}
)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${catkin_INCLUDE_DIRS})


add_executable(joystick_interrupt src/joystick_interrupt.cpp)
target_link_libraries(joystick_interrupt ${catkin_LIBRARIES})

add_executable(joystick_mux src/joystick_mux.cpp)
target_link_libraries(joystick_mux ${catkin_LIBRARIES})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
    joystick_interrupt
    joystick_mux
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

ament_package()

