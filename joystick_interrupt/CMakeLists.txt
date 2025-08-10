cmake_minimum_required(VERSION 3.14.4)
project(joystick_interrupt)

set(AMENT_CMAKE_DEPENDS
  rclcpp

  geometry_msgs
  sensor_msgs
  topic_tools
)

find_package(ament_cmake REQUIRED)
foreach(AMENT_DEPEND ${AMENT_CMAKE_DEPENDS})
  find_package(${AMENT_DEPEND} REQUIRED)
endforeach()


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${catkin_INCLUDE_DIRS})


add_executable(joystick_interrupt src/joystick_interrupt.cpp)
target_link_libraries(joystick_interrupt rclcpp::rclcpp ${sensor_msgs_TARGETS})

# ! No migration found for topic_tools::ShapeFilter
# add_executable(joystick_mux src/joystick_mux.cpp)
# target_link_libraries(joystick_mux rclcpp::rclcpp ${sensor_msgs_TARGETS})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
    joystick_interrupt
    # joystick_mux
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()