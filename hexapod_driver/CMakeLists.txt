cmake_minimum_required(VERSION 2.8.3)
project(hexapod_driver)

# Tell CMake where to find "FindXXX.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
find_package(catkin QUIET COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  trajectory_msgs
  control_msgs
  actionlib_msgs
  actionlib
  tf
  geometry_msgs
  dynamixel_control_hw
  trac_ik_lib
  hexapod_ik
)

find_package(orocos_kdl QUIET)
find_package(HexapodController QUIET)

##Needed for ros packages
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    dynamixel_control_hw
    std_msgs
    std_srvs
    trajectory_msgs
    control_msgs
    actionlib_msgs
    actionlib
    tf
    geometry_msgs
    trac_ik_lib
    hexapod_ik
)

if (NOT ${catkin_FOUND})
  message(FATAL_ERROR "Catkin not found!")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${HexapodController_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

# Basic example
add_executable(hexapod_driver src/hexapod_driver.cpp src/hexapod.cpp)
target_link_libraries(hexapod_driver ${catkin_LIBRARIES})
add_dependencies(hexapod_driver ${catkin_EXPORTED_TARGETS})


# Hexapod control library
add_library(hexapod_control src/hexapod.cpp)
target_link_libraries(hexapod_control ${catkin_LIBRARIES})
add_dependencies(hexapod_control ${catkin_EXPORTED_TARGETS})

# Add cartesian controllers if found
if (${orocos_kdl_FOUND} AND ${trac_ik_lib_FOUND})
  add_executable(hexapod_driver_cartesian
    src/hexapod_driver_cartesian.cpp
    src/hexapod.cpp
    src/hexapod_cartesian.cpp
  )
  target_link_libraries(hexapod_driver_cartesian ${catkin_LIBRARIES})
  add_dependencies(hexapod_driver_cartesian ${catkin_EXPORTED_TARGETS})

  add_library(hexapod_control_cartesian  src/hexapod.cpp src/hexapod_cartesian.cpp)
  target_link_libraries(hexapod_control_cartesian ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
  add_dependencies(hexapod_control_cartesian ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS hexapod_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS hexapod_control
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install these files only if they were compiled
if (${orocos_kdl_FOUND} AND ${trac_ik_lib_FOUND})
  install(TARGETS hexapod_driver_cartesian
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(TARGETS hexapod_control_cartesian
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
endif()

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# install(DIRECTORY config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
