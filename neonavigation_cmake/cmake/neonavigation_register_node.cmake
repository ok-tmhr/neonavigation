
macro(neonavigation_register_node target)

  cmake_parse_arguments(ARG "" "PLUGIN" "LINK_LIBRARIES" ${ARGN})

  set(_libs rclcpp::rclcpp rclcpp_components::component ${ARG_LINK_LIBRARIES})
  neonavigation_add_library(${target}_component ${ARG_UNPARSED_ARGUMENTS} LINK_LIBRARIES ${_libs})

  rclcpp_components_register_node(${target}_component PLUGIN ${ARG_PLUGIN} EXECUTABLE ${target})
endmacro()