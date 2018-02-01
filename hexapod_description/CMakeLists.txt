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
  # generate urdf for ROS (i.e., with package URIs)
  # remove .xacro extension
  string(REGEX REPLACE "(.*)[.]xacro$" "\\1.urdf" ros_urdf ${it})
  # get the name of the file without paths
  get_filename_component(filename ${ros_urdf} NAME)
  # set the desired destination
  set(ros_urdf "${CMAKE_CURRENT_SOURCE_DIR}/${filename}")

  # generate urdf for other simulators (i.e., no package URIs)
  # remove .xacro extension
  string(REGEX REPLACE "(.*)[.]xacro$" "\\1_simulation.urdf" simulation_urdf ${it})
  # get the name of the file without paths
  get_filename_component(filename ${simulation_urdf} NAME)
  # set the desired destination
  set(simulation_urdf "${CMAKE_CURRENT_SOURCE_DIR}/${filename}")

  # create a rule to generate ${ros_urdf} from {it}
  # use the ros-dependant "package://" scheme
  xacro_add_xacro_file(${it} INORDER ${ros_urdf} REMAP path_generation:=package)
  list(APPEND urdf_files ${ros_urdf})

  # create a rule to generate ${simulation_urdf} from {it}
  # path_generation:=relative makes the paths to the meshes be relative and not use the
  # ros-dependant "package://" scheme
  xacro_add_xacro_file(${it} INORDER ${simulation_urdf} REMAP path_generation:=relative)
  list(APPEND urdf_files ${simulation_urdf})
endforeach(it)

# add an abstract target to actually trigger the builds
add_custom_target(generate_urdfs DEPENDS ${urdf_files})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
