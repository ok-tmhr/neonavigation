macro(neonavigation_package)

  cmake_parse_arguments(ARG "" "" "INSTALL_TO_SHARE" ${ARGN})

  if(${PROJECT_NAME}_LIBRARIES)
    install(TARGETS
      ${${PROJECT_NAME}_LIBRARIES}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
  endif()

  if(${PROJECT_NAME}_EXECUTABLES)
    install(TARGETS ${${PROJECT_NAME}_EXECUTABLES}
      DESTINATION lib/${PROJECT_NAME}
    )
  endif()

  if(ARG_INSTALL_TO_SHARE)
    install(DIRECTORY ${ARG_INSTALL_TO_SHARE}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()

  ament_package(${ARG_UNPARSED_ARGUMENTS})

endmacro()