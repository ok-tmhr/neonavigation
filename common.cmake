
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_CXX_EXTENSIONS)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wshadow)
endif()

# Binary installed pcl provided by Linux distro is built with -march=native
# which causes a lot of compatibility problems.
# Define PCL_NO_PRECOMPILE to disable using the binary version.
add_definitions(-DPCL_NO_PRECOMPILE)


macro(rclcpp_components_auto_register_node executable)

  cmake_parse_arguments(ARG "" "PLUGIN;PARAMETERS" "" ${ARGN})

  if("${ARG_PLUGIN}" STREQUAL "")
    message(FATAL_ERROR "rclcpp_components_auto_register_node macro requires a PLUGIN argument for target ${executable}")
  endif()

  ament_auto_add_library(${executable}_component ${ARG_UNPARSED_ARGUMENTS})

  if(ARG_PARAMETERS)
    generate_parameter_library(${executable}_parameters ${ARG_PARAMETERS})
    target_link_libraries(${executable}_component ${executable}_parameters)
  endif()

  rclcpp_components_register_node(
    ${executable}_component
    PLUGIN ${ARG_PLUGIN}
    EXECUTABLE ${executable}
  )

endmacro()
