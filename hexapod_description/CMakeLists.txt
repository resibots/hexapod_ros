cmake_minimum_required(VERSION 2.8.3)

project(hexapod_description)

## Find catkin
find_package(catkin REQUIRED)

##Needed for ros packages
catkin_package(CATKIN_DEPENDS xacro)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
