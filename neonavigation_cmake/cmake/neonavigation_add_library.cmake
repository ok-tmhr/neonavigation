
macro(neonavigation_add_library target)

  cmake_parse_arguments(ARG "NO_TARGET_INCLUDE_DIRECTORIES;NO_PRIVATE" "PARAMETER" "LINK_LIBRARIES;DEPENDENCIES" ${ARGN})

  add_library(${target} SHARED ${ARG_UNPARSED_ARGUMENTS})

  if(NOT ARG_NO_TARGET_INCLUDE_DIRECTORIES)
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  set(_libs "")

  if(ARG_PARAMETER)
    generate_parameter_library(${target}_parameters ${ARG_PARAMETER})
    target_link_libraries(${target} PRIVATE ${target}_parameters)
  endif()

  foreach(_pkg IN LISTS ARG_LINK_LIBRARIES)
    if(_pkg MATCHES "(.+)::(.+)")
      list(APPEND _libs ${_pkg})
    elseif(_pkg MATCHES "(_msgs|_srvs|_interfaces)$")
      list(APPEND _libs ${${_pkg}_TARGETS})
    else()
      list(APPEND _libs ${_pkg})
    endif()
  endforeach()

  if(ARG_NO_PRIVATE)
      target_link_libraries(${target} PUBLIC ${_libs})
    else()
      target_link_libraries(${target} PRIVATE ${_libs})
  endif()

  if(ARG_DEPENDENCIES)
    ament_target_dependencies(${target} PUBLIC ${ARG_DEPENDENCIES})
  endif()

  list(APPEND ${PROJECT_NAME}_LIBRARIES ${target})

endmacro()