
macro(neonavigation_find_build_dependencies)
  cmake_parse_arguments(ARG "" "" "EXCLUDE" ${ARGN})

  if(NOT _AMENT_PACKAGE_NAME)
    ament_package_xml()
  endif()

  foreach(_dep IN LISTS ${PROJECT_NAME}_BUILD_DEPENDS ${PROJECT_NAME}_BUILDTOOL_DEPENDS)
    if(NOT _dep IN_LIST ARG_EXCLUDE)
      find_package(${_dep} REQUIRED)
    endif()
  endforeach()
endmacro()