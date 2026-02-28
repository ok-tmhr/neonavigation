
macro(neonavigation_add_library target)

  cmake_parse_arguments(ARG "NO_TARGET_INCLUDE_DIRECTORIES;NO_PRIVATE" "" "LINK" ${ARGN})

  add_library(${target} SHARED ${ARG_UNPARSED_ARGUMENTS})

  if(NOT ARG_NO_TARGET_INCLUDE_DIRECTORIES)
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  set(_libs rclcpp::rclcpp rclcpp_components::component)
  foreach(_pkg IN LISTS ARG_LINK)
    if(_pkg MATCHES "(_msgs|_srvs|_interfaces)$")
      list(APPEND _libs ${${_pkg}_TARGETS})
    else()
      list(APPEND _libs ${_pkg})
    endif()
  endforeach()

  if(ARG_NO_PRIVATE)
      target_link_libraries(${target} ${_libs})
    else()
      target_link_libraries(${target} PRIVATE ${_libs})
  endif()

  list(APPEND ${PROJECT_NAME}_LIBRARIES ${target})

endmacro()