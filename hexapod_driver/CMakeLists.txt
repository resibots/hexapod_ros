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
  trajectory_msgs
  control_msgs
  actionlib_msgs
  actionlib
)

find_package(HexapodController QUIET COMPONENTS hexapod_controller_simple)

##Needed for ros packages
catkin_package(CATKIN_DEPENDS roscpp rospy dynamixel_control_hw std_msgs trajectory_msgs control_msgs actionlib_msgs actionlib)

include_directories(include ${catkin_INCLUDE_DIRS} ${HexapodController_INCLUDE_DIRS})
add_executable(hexapod_driver src/hexapod_driver.cpp src/hexapod.cpp)
target_link_libraries(hexapod_driver ${catkin_LIBRARIES} ${HexapodController_LIBRARIES})
add_dependencies(hexapod_driver ${catkin_EXPORTED_TARGETS})

install(TARGETS hexapod_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# install(DIRECTORY config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
