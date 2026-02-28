
macro(neonavigation_register_node target)

  cmake_parse_arguments(ARG "" "PARAMETER;PLUGIN" "" ${ARGN})

  neonavigation_add_library(${target}_component ${ARG_UNPARSED_ARGUMENTS})

  if(ARG_PARAMETER)
    generate_parameter_library(${target}_parameters ${ARG_PARAMETER})
    target_link_libraries(${target}_component PRIVATE ${target}_parameters)
  endif()

  rclcpp_components_register_node(${target}_component PLUGIN ${ARG_PLUGIN} EXECUTABLE ${target})
endmacro()