cmake_minimum_required(VERSION 2.8.3)
project(hexapod_driver)

# Tell CMake where to find "FindXXX.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
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
)

find_package(HexapodController QUIET) # COMPONENTS hexapod_controller_simple)

##Needed for ros packages
catkin_package(CATKIN_DEPENDS roscpp rospy dynamixel_control_hw std_msgs std_srvs trajectory_msgs control_msgs actionlib_msgs actionlib tf geometry_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${HexapodController_INCLUDE_DIRS})
add_executable(hexapod_driver src/hexapod_driver.cpp src/hexapod.cpp)
target_link_libraries(hexapod_driver ${catkin_LIBRARIES})
add_dependencies(hexapod_driver ${catkin_EXPORTED_TARGETS})

add_executable(read_state src/read_state.cpp)
target_link_libraries(read_state ${catkin_LIBRARIES})
add_dependencies(read_state ${catkin_EXPORTED_TARGETS})

add_library(hexapod_control src/hexapod.cpp)
target_link_libraries(hexapod_control ${catkin_LIBRARIES})
add_dependencies(hexapod_control ${catkin_EXPORTED_TARGETS})

install(TARGETS read_state
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS hexapod_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS hexapod_control
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# install(DIRECTORY config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
