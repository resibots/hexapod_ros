cmake_minimum_required(VERSION 2.8.3)
project(hexapod_controllers)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_msgs
  std_msgs
  std_srvs
  control_toolbox
  controller_interface
  hardware_interface
  realtime_tools
  velocity_controllers
  message_generation
  geometry_msgs
)


# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    control_msgs
    std_msgs
    std_srvs
    control_toolbox
    controller_interface
    hardware_interface
    realtime_tools
    velocity_controllers
    message_runtime
    geometry_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include  ${catkin_INCLUDE_DIRS} )

add_library(${PROJECT_NAME}
  src/open_loop_cpg_controller.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES hexapod_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
