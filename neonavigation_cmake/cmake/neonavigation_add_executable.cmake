
macro(neonavigation_add_executable target)

  cmake_parse_arguments(ARG "NO_TARGET_INCLUDE_DIRECTORIES;NO_PRIVATE" "" "LINK" ${ARGN})

  add_executable(${target} ${ARG_UNPARSED_ARGUMENTS})

  if(NOT ARG_NO_TARGET_INCLUDE_DIRECTORIES)
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  set(_libs "")
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

  list(APPEND ${PROJECT_NAME}_EXECUTABLES ${target})

endmacro()