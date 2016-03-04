cmake_minimum_required(VERSION 2.8.3)

#These are Hexapod Controller's known components (ie. libraries)
# set(HexapodController_COMPONENTS
#     hexapod_controller_simple
#     )

#Set INCLUDE hints
set(HexapodController_INCLUDE_HINTS
    "${RESIBOTS_DIR}/include"
    "$ENV{RESIBOTS_DIR}/include" )

# Set LIBRARY hints
# set(HexapodController_LIBRARY_HINTS
#     "${RESIBOTS_DIR}/lib"
#     "$ENV{RESIBOTS_DIR}/lib" )

# Find include directories
find_path(HexapodController_INCLUDE_DIR hexapod_controller/hexapod_controller_simple.hpp HINTS ${HexapodController_INCLUDE_HINTS} )

# Verify we know about all the components requested
# and remove those we don't know about
# set(HexapodController_FILTERED_COMPONENTS ${HexapodController_FIND_COMPONENTS})
#
# if ( HexapodController_FIND_COMPONENTS )
#     foreach(comp ${HexapodController_FIND_COMPONENTS})
#         list(FIND HexapodController_COMPONENTS ${comp} ${comp}_KNOWN)
#         if (${comp}_KNOWN EQUAL -1)
#             list(REMOVE_ITEM HexapodController_FILTERED_COMPONENTS ${comp})
#             message(STATUS "Unknown HexapodController component ${comp}")
#         endif()
#     endforeach()
# endif()
#
# list(LENGTH HexapodController_FILTERED_COMPONENTS HexapodController_NUMBER_OF_COMPONENTS)
# set(HexapodController_FOUND_COMPONENTS TRUE)
#
# # Look for components (ie. libraries)
# if( ${HexapodController_NUMBER_OF_COMPONENTS}  )
#     foreach(comp ${HexapodController_FILTERED_COMPONENTS})
#         #Look for the actual library here
#         find_library(${comp}_LIBRARY NAMES ${comp} HINTS ${HexapodController_LIBRARY_HINTS})
#         if ( ${${comp}_LIBRARY} STREQUAL ${comp}_LIBRARY-NOTFOUND)
#             message(STATUS "Could not find NAOqi's ${comp}")
#             set(HexapodController_FOUND_COMPONENTS FALSE)
#         else()
#             #If everything went well append this component to list of libraries
#             list(APPEND HexapodController_LIBRARY ${${comp}_LIBRARY})
#         endif()
#     endforeach()
# else()
#     message(STATUS "No HexapodController components specified")
# endif()


# Handle the QUIET and REQUIRED arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    HexapodController #Package name
    DEFAULT_MSG
    # Variables required to evaluate as TRUE
    # HexapodController_LIBRARY
  HexapodController_INCLUDE_DIR)
    # HexapodController_FOUND_COMPONENTS)

# Copy the values of the advanced variables to the user-facing ones
# set(HexapodController_LIBRARIES ${HexapodController_LIBRARY} )
set(HexapodController_INCLUDE_DIRS ${HexapodController_INCLUDE_DIR} )
set(HexapodController_FOUND ${HEXAPODCONTROLLER_FOUND})

# If NAOqi was found, update NAOqi_DIR to show where it was found
if ( HexapodController_FOUND )
  get_filename_component(HexapodController_NEW_DIR "${HexapodController_INCLUDE_DIRS}/../" ABSOLUTE)
endif()
set(HexapodController_DIR ${HexapodController_NEW_DIR} CACHE FILEPATH "HexapodController root directory" FORCE)

#Hide these variables
mark_as_advanced(HexapodController_INCLUDE_DIR HexapodController_LIBRARY HEXAPODCONTROLLER_FOUND)
