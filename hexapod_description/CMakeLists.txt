cmake_minimum_required(VERSION 2.8.3)

project(hexapod_description)

## Find catkin
find_package(catkin REQUIRED COMPONENTS xacro)

## Needed for ros packages
catkin_package(CATKIN_DEPENDS xacro)

# Xacro files
list(APPEND xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/urdf/hexaforce.xacro)
list(APPEND xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/urdf/pexod.xacro)

foreach(it ${xacro_files})
  # remove .xacro extension
  string(REGEX REPLACE "(.*)[.]xacro$" "\\1_simulation.urdf" output_filename ${it})

  # create a rule to generate ${output_filename} from {it}
  # path_generation:=relative makes the paths to the meshes be relative and not use the
  # ros-dependant "package://" scheme
  xacro_add_xacro_file(${it} INORDER ${output_filename} REMAP path_generation:=relative)
  # xacro_add_xacro_file(${it} INORDER REMAP path_generation:=relative)

  list(APPEND urdf_files ${output_filename})
endforeach(it)

# add an abstract target to actually trigger the builds
add_custom_target(generate_urdfs DEPENDS ${urdf_files})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
