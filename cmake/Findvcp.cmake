# - Try to find vito
# Once done this will define
#  VCP_FOUND 
#  VCP_INCLUDE_DIR - The include directories
#  VCP_LIBRARIES - The libraries
#
# If the environmental variable VCP_ROOT_DIR is NOT set, this script will try to
# locate VCP via relative imports

##############################################################################
# Where's the library's root dir?
if(DEFINED ENV{VCP_ROOT_DIR})
    if(WIN32)
        string(REPLACE "\\" "/" VCP_ROOT $ENV{VCP_ROOT_DIR})
    else(WIN32)
        set(VCP_ROOT $ENV{VCP_ROOT_DIR})
    endif(WIN32)
else()
    set(VCP_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
    if(NOT vcp_FIND_QUIETLY)
        message(STATUS "[vcp] Environmental variable VCP_ROOT_DIR not set! Assuming relative to Findvcp.cmake: '${VCP_ROOT}'")
    endif()
endif()

##############################################################################
# Set up hints for file searches
set(POTENTIAL_INCLUDE_DIR "${VCP_ROOT}/gen/include")
set(POTENTIAL_LIBRARY_PATHS "${VCP_ROOT}/gen/lib" "${VCP_ROOT}/gen/lib64")

##############################################################################
# Always include the utils module.
set(VCP_MODULES "vcp_utils")

##############################################################################
# Add additional modules requested within find_package:
if(vcp_FIND_COMPONENTS)
    set(VCP_MOD_FIND_MSG "Selected")
    foreach(component ${vcp_FIND_COMPONENTS})
        string(TOLOWER ${component} _COMPONENT)
        set(VCP_MODULES ${VCP_MODULES} ${_COMPONENT})
    endforeach()
else()
    # Add all modules by default
    set(VCP_MOD_FIND_MSG "Configured all")
    set(VCP_MODULES ${VCP_MODULES} vcp_best vcp_bgm vcp_config vcp_imutils vcp_imvis vcp_math vcp_ui)
endif()

##############################################################################
# Set up file name hints (postfix for static/debug versions)
set(QUERY_STRING_STATIC_LIB "_static")
set(QUERY_STRING_DEBUG_LIB "d")


##############################################################################
# First, find out which modules the user requested
list(FIND VCP_MODULES vcp_best      REQUESTED_BEST)
list(FIND VCP_MODULES vcp_bgm       REQUESTED_BGM)
    # list(FIND VCP_MODULES vcp_config    REQUESTED_CONFIG)  # Only requires vcp_utils
list(FIND VCP_MODULES vcp_imutils   REQUESTED_IMUTILS)
list(FIND VCP_MODULES vcp_imvis     REQUESTED_IMVIS)
    # list(FIND VCP_MODULES vcp_math      REQUESTED_MATH)  # No dependencies (uses header-only def's of other modules if needed)
list(FIND VCP_MODULES vcp_tracking  REQUESTED_TRACKING)
    # list(FIND VCP_MODULES vcp_ui        REQUESTED_UI)  # Only requires vcp_utils

# ... then, add dependencies (e.g. vcp_imvis uses vcp_math)
if(REQUESTED_BEST GREATER 0)
    set(VCP_MODULES ${VCP_MODULES} vcp_config vcp_imutils)
    message(WARNING "Need to update package list!")
endif()
if(REQUESTED_BGM GREATER 0)
    set(VCP_MODULES ${VCP_MODULES} vcp_imutils vcp_math)
endif()
if(REQUESTED_IMUTILS GREATER 0)
    set(VCP_MODULES ${VCP_MODULES} vcp_math)
endif()
if(REQUESTED_TRACKING GREATER 0)
    message(FATAL_ERROR "Not yet supported")
endif()
if(REQUESTED_IMVIS GREATER 0)
    set(VCP_MODULES ${VCP_MODULES} vcp_imutils vcp_math)
endif()


##############################################################################
# Query each requested/required module.
list(REMOVE_DUPLICATES VCP_MODULES)
foreach(module ${VCP_MODULES})
    # Select order of names to search for libs (prefer static or shared depending on configuration).
    if(VCP_PREFER_STATIC_LIBRARIES)
        set(VCP_NAMES_DEBUG   ${module}${QUERY_STRING_STATIC_LIB}${QUERY_STRING_DEBUG_LIB} ${module}${QUERY_STRING_DEBUG_LIB})
        set(VCP_NAMES_RELEASE ${module}${QUERY_STRING_STATIC_LIB} ${module})
    else()
        set(VCP_NAMES_DEBUG   ${module}${QUERY_STRING_DEBUG_LIB} ${module}${QUERY_STRING_STATIC_LIB}${QUERY_STRING_DEBUG_LIB})
        set(VCP_NAMES_RELEASE ${module} ${module}${QUERY_STRING_STATIC_LIB})
    endif()

    # Try to find release/debug libraries
    unset(LIBRARY_RELEASE CACHE)
    unset(LIBRARY_DEBUG CACHE)

    find_library(LIBRARY_RELEASE NAMES ${VCP_NAMES_RELEASE}
      HINTS ${POTENTIAL_LIBRARY_PATHS}
      NO_DEFAULT_PATH
    )
    find_library(LIBRARY_DEBUG NAMES ${VCP_NAMES_DEBUG}
      HINTS ${POTENTIAL_LIBRARY_PATHS}
      NO_DEFAULT_PATH
    )

    # Abort if no library found
    if(NOT LIBRARY_DEBUG AND NOT LIBRARY_RELEASE AND vcp_FIND_REQUIRED)
        message(FATAL_ERROR "[vcp] ${module} marked as required but not found")
    endif()

    # If we found the debug version, use it (will be replaced by the
    # release version directly below).
    if(LIBRARY_DEBUG)
        set(CURRENT_LIBRARY ${LIBRARY_DEBUG})
    endif()
    # Prefer release build if available.
    if(LIBRARY_RELEASE)
        set(CURRENT_LIBRARY ${LIBRARY_RELEASE})
    endif()

    # Derive the library directory from the selected library.
    get_filename_component(_LIB_DIR "${CURRENT_LIBRARY}" PATH)
    get_filename_component(VCP_LIBRARY_DIR "${_LIB_DIR}" REALPATH)

    # Append this module's library to the list
    set(VCP_LIBRARIES ${VCP_LIBRARIES} ${CURRENT_LIBRARY})
endforeach()


##############################################################################
# Add external dependencies for the selected/requested modules
list(FIND VCP_MODULES vcp_best      USE_BEST)
list(FIND VCP_MODULES vcp_bgm       USE_BGM)
list(FIND VCP_MODULES vcp_config    USE_CONFIG)
list(FIND VCP_MODULES vcp_imutils   USE_IMUTILS)
list(FIND VCP_MODULES vcp_imvis     USE_IMVIS)
list(FIND VCP_MODULES vcp_math      USE_MATH)
list(FIND VCP_MODULES vcp_tracking  USE_TRACKING)
list(FIND VCP_MODULES vcp_ui        USE_UI)


# OpenCV
if(USE_BEST GREATER 0 OR USE_BGM GREATER 0 OR USE_IMUTILS GREATER 0 OR USE_MATH GREATER 0 OR USE_TRACKING GREATER 0 OR USE_UI GREATER 0 OR USE_IMVIS GREATER 0)
    if(NOT vcp_FIND_QUIETLY)
        message(STATUS "[vcp] Some of the selected VCP modules require OpenCV")
    endif()

    find_package(OpenCV REQUIRED)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "[vcp] OpenCV not found!")
    endif()

    # Add OpenCV libraries to VCP library list
    set(VCP_LIBRARIES ${VCP_LIBRARIES} ${OpenCV_LIBS})
endif()


# CURL
if(USE_BEST GREATER 0)
    if(NOT vcp_FIND_QUIETLY)
        message(STATUS "[vcp] vcp_best requires CURL")
    endif()

    find_package(CURL REQUIRED)
    if(NOT CURL_FOUND)
        message(FATAL_ERROR "[vcp] CURL not found!")
    endif()

    # Add CURL libraries to VCP library list
    set(VCP_LIBRARIES ${VCP_LIBRARIES} ${CURL_LIBRARIES})

    # Check if matrix vision sink was installed, if so, add mvIMPACT
    if (EXISTS "${VCP_ROOT}/gen/include/vcp_icc/matrixvision_sink.h")
		    message(STATUS "[vcp] vcp_best was built with matrix vision support, thus requires mvIMPACT SDK")
        find_package(mvIMPACT_acquire REQUIRED)
	      include_directories(${mvIMPACT_acquire_INCLUDE_DIR})
        set(VCP_LIBRARIES ${VCP_LIBRARIES} ${mvIMPACT_acquire_LIBS})
    endif()

    # Check if realsense2 sink was installed, if so, add realsense2
    if (EXISTS "${VCP_ROOT}/gen/include/vcp_icc/realsense2_sink.h")
        message(STATUS "[vcp] vcp_best was built with RealSense2 support, thus requires librealsense2")
        find_package(realsense2 REQUIRED)
	      include_directories(${realsense_INCLUDE_DIR})
        set(VCP_LIBRARIES ${VCP_LIBRARIES} ${realsense2_LIBRARY})
    endif()

    # Check if k4a sink was installed, if so, add k4a dependency
    if (EXISTS "${VCP_ROOT}/gen/include/vcp_icc/k4a_sink.h")
		  message(STATUS "[vcp] vcp_best was built with Azure Kinect support, thus requires libk4a")
      find_package(k4a REQUIRED)
      set(VCP_LIBRARIES ${VCP_LIBRARIES} k4a)
    endif()
endif()


# zlib
if(USE_IMUTILS GREATER 0)
    if(NOT vcp_FIND_QUIETLY)
        message(STATUS "[vcp] vcp_imutils was linked against zlib, thus requires this dependency")
    endif()

    find_package(ZLIB REQUIRED)
    if(NOT ZLIB_FOUND)
        message(FATAL_ERROR "[vcp] zlib not found!")
    endif()
    set(VCP_LIBRARIES ${VCP_LIBRARIES} ${ZLIB_LIBRARIES})
endif()

# libconfig++
if(USE_CONFIG GREATER 0)
    if(NOT vcp_FIND_QUIETLY)
        message(STATUS "[vcp] vcp_config requires libconfig++")
    endif()

    find_package(LibConfig++ REQUIRED)
    if(NOT LIBCONFIG++_FOUND)
        message(FATAL_ERROR "[vcp] libconfig++ not found!")
    endif()

    # Add CURL libraries to VCP library list
    set(VCP_LIBRARIES ${VCP_LIBRARIES} ${LIBCONFIG++_LIBRARY})
endif()


##############################################################################
# Find include directory (by searching for vcp_utils/string_utils.h)
find_path(VCP_UTILS_INCLUDE_DIR string_utils.h
    HINTS ${POTENTIAL_INCLUDE_DIR}/vcp_utils
    NO_DEFAULT_PATH
)

if(VCP_UTILS_INCLUDE_DIR AND VCP_LIBRARIES)
    set(VCP_FOUND TRUE)
    # Set include directory to parent dir, s.t. we can "#include <vcp_utils/xyz.h>".
    get_filename_component(VCP_INCLUDE_DIR "${VCP_UTILS_INCLUDE_DIR}/.." REALPATH)
endif()


##############################################################################
# Clean up, raise error if required, notify if we're not forced to be quiet...
if(VCP_FOUND)
  if(NOT vcp_FIND_QUIETLY)
    message(STATUS "[vcp] Include directory: ${VCP_INCLUDE_DIR}")
    message(STATUS "[vcp] ${VCP_MOD_FIND_MSG} modules:  ${VCP_MODULES}")
#message(STATUS "[vcp] Libraries: ${VCP_LIBRARIES}")
  endif()
else()
  if(vcp_FIND_REQUIRED)
    message(FATAL_ERROR "[vcp] Could not locate selected VCP modules")
  endif()
endif()

