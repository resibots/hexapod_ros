cmake_minimum_required(VERSION 2.8.3)

#Set INCLUDE hints
set(HexapodController_INCLUDE_HINTS
    "${RESIBOTS_DIR}/include"
    "$ENV{RESIBOTS_DIR}/include" )

# Find include directories
find_path(HexapodController_INCLUDE_DIR hexapod_controller/hexapod_controller_simple.hpp HINTS ${HexapodController_INCLUDE_HINTS} )


# Handle the QUIET and REQUIRED arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    HexapodController #Package name
    DEFAULT_MSG
  HexapodController_INCLUDE_DIR)

# Copy the values of the advanced variables to the user-facing ones
# set(HexapodController_LIBRARIES ${HexapodController_LIBRARY} )
set(HexapodController_INCLUDE_DIRS ${HexapodController_INCLUDE_DIR} )
set(HexapodController_FOUND ${HEXAPODCONTROLLER_FOUND})

# If HexapodController was found, update HexapodController_DIR to show where it was found
if ( HexapodController_FOUND )
  get_filename_component(HexapodController_NEW_DIR "${HexapodController_INCLUDE_DIRS}/../" ABSOLUTE)
endif()
set(HexapodController_DIR ${HexapodController_NEW_DIR} CACHE FILEPATH "HexapodController root directory" FORCE)

#Hide these variables
mark_as_advanced(HexapodController_INCLUDE_DIR HexapodController_LIBRARY HEXAPODCONTROLLER_FOUND)
