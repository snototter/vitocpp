# - Try to find pvt_utils
# Once done this will define
#  PVT_UTILS_FOUND 
#  PVT_UTILS_INCLUDE_DIRS - The pvt_utils include directories
#  PVT_UTILS_LIBRARIES - The pvt_utils libraries
#  PVT_UTILS_LIBRARY_DIR - The pvt_utils library directory (note that PVT_UTILS_LIBRARIES contain full paths)

set(QUERY_PVT_UTILS_NAME "pvt_utils")
set(QUERY_STATIC_LIB_STRING "_static")
set(QUERY_DEBUG_POSTFIX "d")
set(POTENTIAL_INCLUDE_PATH $ENV{PVT_ROOT_DIR}/install/include/pvt/utils ${CMAKE_SOURCE_DIR}/install/include/pvt/utils)
set(POTENTIAL_LIBRARY_PATH $ENV{PVT_ROOT_DIR}/install/lib ${CMAKE_SOURCE_DIR}/install/lib)

if(PVT_PREFER_STATIC_LIBRARIES)
  if(NOT pvt_utils_FIND_QUIETLY)
    message("Looking for the static pvt_utils library")
  endif()
  set(PVT_UTILS_NAMES_DEBUG   ${QUERY_PVT_UTILS_NAME}${QUERY_STATIC_LIB_STRING}${QUERY_DEBUG_POSTFIX})
  set(PVT_UTILS_NAMES_RELEASE ${QUERY_PVT_UTILS_NAME}${QUERY_STATIC_LIB_STRING})
else()
  if(NOT pvt_utils_FIND_QUIETLY)
    message("Looking for the shared pvt_utils library")
  endif()
  set(PVT_UTILS_NAMES_DEBUG   ${QUERY_PVT_UTILS_NAME}${QUERY_DEBUG_POSTFIX})
  set(PVT_UTILS_NAMES_RELEASE ${QUERY_PVT_UTILS_NAME})
endif()

# Find include directory
find_path(PVT_UTILS_INCLUDE_DIR string_utils.h
  HINTS ${POTENTIAL_INCLUDE_PATH}
  NO_DEFAULT_PATH
)

# Try to find release/debug libraries
find_library(PVT_UTILS_LIBRARY_RELEASE NAMES ${PVT_UTILS_NAMES_RELEASE}
  HINTS ${POTENTIAL_LIBRARY_PATH}
  NO_DEFAULT_PATH
)
find_library(PVT_UTILS_LIBRARY_DEBUG NAMES ${PVT_UTILS_NAMES_DEBUG}
  HINTS ${POTENTIAL_LIBRARY_PATH}
  NO_DEFAULT_PATH
)

# Abort if no library found
if(NOT PVT_UTILS_LIBRARY_DEBUG AND NOT PVT_UTILS_LIBRARY_RELEASE AND pvt_utils_FIND_REQUIRED)
  message(FATAL_ERROR "pvt_utils marked as required but not found")
endif()

# If we found the debug version, use it (may be replaced by release version below).
if(PVT_UTILS_LIBRARY_DEBUG)
  set(OUTPUT_LIBRARY ${PVT_UTILS_LIBRARY_RELEASE})
endif()
# Prefer release build if available.
if(PVT_UTILS_LIBRARY_RELEASE)
  set(OUTPUT_LIBRARY ${PVT_UTILS_LIBRARY_RELEASE})
endif()

# Derive the library directory from the selected library
get_filename_component(PVT_UTILS_LIBRARY_DIR "${OUTPUT_LIBRARY}" REALPATH)

# Set PVT_UTILS_LIBRARIES variable
if(OUTPUT_LIBRARY)
  set(PVT_UTILS_FOUND TRUE)

  set(PVT_UTILS_LIBRARIES ${OUTPUT_LIBRARY})
endif()

if(PVT_UTILS_INCLUDE_DIR)
  # Set include directory to parent dir, s.t. we can include<pvt_utils/xyz.h>
  get_filename_component(PVT_UTILS_INCLUDE_DIRS "${PVT_UTILS_INCLUDE_DIR}/.." REALPATH)
endif()

# Message
if(PVT_UTILS_FOUND)
  if(NOT pvt_utils_FIND_QUIETLY)
    message(STATUS "Found pvt_utils: ${PVT_UTILS_LIBRARIES} ${PVT_UTILS_INCLUDE_DIRS}")
  endif()
else()
  if(pvt_utils_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find pvt_utils")
  endif()
endif()



