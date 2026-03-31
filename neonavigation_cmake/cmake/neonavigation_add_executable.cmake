
macro(neonavigation_add_executable target)

  cmake_parse_arguments(ARG "NO_TARGET_INCLUDE_DIRECTORIES;NO_PRIVATE" "" "LINK_LIBRARIES;DEPENDENCIES" ${ARGN})

  add_executable(${target} ${ARG_UNPARSED_ARGUMENTS})

  if(NOT ARG_NO_TARGET_INCLUDE_DIRECTORIES)
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  set(_libs "")
  foreach(_pkg IN LISTS ARG_LINK_LIBRARIES)
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

  if(ARG_DEPENDENCIES)
    if (ARG_NO_PRIVATE)
      ament_target_dependencies(${target} ${ARG_DEPENDENCIES})
    else()
      ament_target_dependencies(${target} PUBLIC ${ARG_DEPENDENCIES})
    endif()
  endif()

  list(APPEND ${PROJECT_NAME}_EXECUTABLES ${target})

endmacro()