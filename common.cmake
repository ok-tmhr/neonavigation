
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wshadow)
endif()

function(rclcpp_components_auto_register_node executable)

  cmake_parse_arguments(ARGS "" "PLUGIN;PARAMETERS" "" ${ARGN})

  if("${ARGS_PLUGIN}" STREQUAL "")
    message(FATAL_ERROR "rclcpp_components_auto_register_node macro requires a PLUGIN argument for target ${executable}")
  endif()

  ament_auto_add_library(${executable}_component ${ARGS_UNPARSED_ARGUMENTS})

  if(ARGS_PARAMETERS)
    generate_parameter_library(${executable}_parameters ${ARGS_PARAMETERS})
    target_link_libraries(${executable}_component ${executable}_parameters)
  endif()

  rclcpp_components_register_node(
    ${executable}_component
    PLUGIN ${ARGS_PLUGIN}
    EXECUTABLE ${executable}
  )

endfunction()
