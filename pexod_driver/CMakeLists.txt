cmake_minimum_required(VERSION 2.8.3)
project(pexod_driver)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamixel_control_hw
)

##Needed for ros packages
catkin_package(CATKIN_DEPENDS roscpp rospy dynamixel_control_hw)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(pexod_driver src/pexod_driver.cpp)
target_link_libraries(pexod_driver ${catkin_LIBRARIES})
add_dependencies(pexod_driver ${catkin_EXPORTED_TARGETS})

install(TARGETS pexod_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# install(DIRECTORY config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
