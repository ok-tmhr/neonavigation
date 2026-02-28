
macro(neonavigation_find_build_dependencies)
  if(NOT _AMENT_PACKAGE_NAME)
    ament_package_xml()
  endif()

  foreach(_dep IN LISTS ${PROJECT_NAME}_BUILD_DEPENDS ${PROJECT_NAME}_BUILDTOOL_DEPENDS)
    find_package(${_dep} REQUIRED)
  endforeach()
endmacro()