# - Try to find vito
# Once done this will define
#  VITOCPP_FOUND 
#  VITOCPP_INCLUDE_DIR - The include directories
#  VITOCPP_LIBRARIES - The libraries
#  VITOCPP_LIBRARY_DIR - Link directories, useful for rpath
# You must set the environmental variable VITOCPP_ROOT_DIR (see installation instructions)!

#TODO automatically add dependent components (if you want pvt_visualization, then you also need pvt_imutils)

if(DEFINED ENV{PVT_ROOT_DIR})
  if(WIN32)
    string(REPLACE "\\" "/" PVT_ROOT $ENV{PVT_ROOT_DIR})
  else(WIN32)
    set(PVT_ROOT $ENV{PVT_ROOT_DIR})
  endif(WIN32)
else(DEFINED ENV{PVT_ROOT_DIR})
  set(PVT_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
  message(WARNING "Environmental variable PVT_ROOT_DIR not set! Trying to continue from ${PVT_ROOT}")
endif(DEFINED ENV{PVT_ROOT_DIR})

set(POTENTIAL_INCLUDE_DIR "${PVT_ROOT}/include")
set(POTENTIAL_LIBRARY_PATHS ${PVT_ROOT}/lib ${PVT_ROOT}/lib64)

# Always include the utils module.
set(PVT_MODULES "pvt_utils")

# Add additional modules requested within find_package:
if(pvt_FIND_COMPONENTS)
  foreach(component ${pvt_FIND_COMPONENTS})
    string(TOLOWER ${component} _COMPONENT)
    set(PVT_MODULES ${PVT_MODULES} ${_COMPONENT})
  endforeach()
endif()

set(QUERY_STRING_STATIC_LIB "_static")
set(QUERY_STRING_DEBUG_LIB "d")

# Query each requested module.
list(REMOVE_DUPLICATES PVT_MODULES)
foreach(module ${PVT_MODULES})
  # Select order of names to search for libs (prefer static or shared depending on configuration).
  if(PVT_PREFER_STATIC_LIBRARIES)
    set(PVT_NAMES_DEBUG   ${module}${QUERY_STRING_STATIC_LIB}${QUERY_STRING_DEBUG_LIB} ${module}${QUERY_STRING_DEBUG_LIB})
    set(PVT_NAMES_RELEASE ${module}${QUERY_STRING_STATIC_LIB} ${module})
  else()
    set(PVT_NAMES_DEBUG   ${module}${QUERY_STRING_DEBUG_LIB} ${module}${QUERY_STRING_STATIC_LIB}${QUERY_STRING_DEBUG_LIB})
    set(PVT_NAMES_RELEASE ${module} ${module}${QUERY_STRING_STATIC_LIB})
  endif()

  # Try to find release/debug libraries
  unset(LIBRARY_RELEASE CACHE)
  unset(LIBRARY_DEBUG CACHE)

  find_library(LIBRARY_RELEASE NAMES ${PVT_NAMES_RELEASE}
    HINTS ${POTENTIAL_LIBRARY_PATHS}
    NO_DEFAULT_PATH
  )
  find_library(LIBRARY_DEBUG NAMES ${PVT_NAMES_DEBUG}
    HINTS ${POTENTIAL_LIBRARY_PATHS}
    NO_DEFAULT_PATH
  )

  # Abort if no library found
  if(NOT LIBRARY_DEBUG AND NOT LIBRARY_RELEASE AND pvt_FIND_REQUIRED)
    message(FATAL_ERROR "${module} marked as required but not found")
  endif()

  # If we found the debug version, use it (may be replaced by release version below).
  if(LIBRARY_DEBUG)
    set(CURRENT_LIBRARY ${LIBRARY_DEBUG})
  endif()
  # Prefer release build if available.
  if(LIBRARY_RELEASE)
    set(CURRENT_LIBRARY ${LIBRARY_RELEASE})
  endif()

  # Derive the library directory from the selected library.
  get_filename_component(_LIB_DIR "${CURRENT_LIBRARY}" PATH)
  get_filename_component(PVT_LIBRARY_DIR "${_LIB_DIR}" REALPATH)

  # Append lib to list
  set(PVT_LIBRARIES ${PVT_LIBRARIES} ${CURRENT_LIBRARY})
endforeach()

# Add dependencies for your selected/requested PVT modules
list(FIND PVT_MODULES pvt_bgm USE_BACKGROUND_MODELS)
list(FIND PVT_MODULES pvt_config USE_CONFIG)
list(FIND PVT_MODULES pvt_icc USE_IP_CAM_CAPTURE)
list(FIND PVT_MODULES pvt_math USE_MATH)
list(FIND PVT_MODULES pvt_imutils USE_IMUTILS)
list(FIND PVT_MODULES pvt_ui USE_UI)
list(FIND PVT_MODULES pvt_visualization USE_VISUALIZATION)
list(FIND PVT_MODULES pvt_tracking USE_TRACKING)

# OpenCV
if(USE_VISUALIZATION GREATER 0 OR USE_TRACKING GREATER 0 OR USE_BACKGROUND_MODELS GREATER 0 OR USE_IP_CAM_CAPTURE GREATER 0 OR USE_UI GREATER 0 OR USE_MATH GREATER 0 OR USE_IMUTILS GREATER 0)
  if(NOT pvt_FIND_QUIETLY)
    message(STATUS "Some selected PVT modules require OpenCV as dependency")
  endif()

  find_package(OpenCV REQUIRED)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found!")
  endif()

  # Add OpenCV libraries to PVT library list
  set(PVT_LIBRARIES ${PVT_LIBRARIES} ${OpenCV_LIBS})
  # TODO do we need to add the OpenCV include directory (for those headers that refer to opencv/core, etc)?
endif()

# CURL
if(USE_IP_CAM_CAPTURE GREATER 0)
  if(NOT pvt_FIND_QUIETLY)
    message(STATUS "Some selected PVT modules require CURL as dependency")
  endif()

  find_package(CURL REQUIRED)
  if(NOT CURL_FOUND)
    message(FATAL_ERROR "CURL not found!")
  endif()

  # Add CURL libraries to PVT library list
  set(PVT_LIBRARIES ${PVT_LIBRARIES} ${CURL_LIBRARIES})

  # Check if matrix vision sink was installed, if so, add mvIMPACT
  if (EXISTS "${PVT_ROOT}/include/pvt_icc/matrixvision_sink.h")
		message(STATUS "PVT ICC was build with matrix vision support, need to include mvIMPACT SDK")
    find_package(mvIMPACT_acquire REQUIRED)
	  include_directories(${mvIMPACT_acquire_INCLUDE_DIR})
    set(PVT_LIBRARIES ${PVT_LIBRARIES} ${mvIMPACT_acquire_LIBS})
  endif()

  # Check if realsense2 sink was installed, if so, add realsense2
  if (EXISTS "${PVT_ROOT}/include/pvt_icc/realsense2_sink.h")
		message(STATUS "PVT ICC was build with realsense2 support, need to link librealsense2")
    find_package(realsense2 REQUIRED)
	  include_directories(${realsense_INCLUDE_DIR})
    set(PVT_LIBRARIES ${PVT_LIBRARIES} ${realsense2_LIBRARY})
  endif()

  # Check if k4a sink was installed, if so, add k4a dependency
  if (EXISTS "${PVT_ROOT}/include/pvt_icc/k4a_sink.h")
		message(STATUS "PVT ICC was build with k4a support, need to link libk4a")
    find_package(k4a REQUIRED)
    set(PVT_LIBRARIES ${PVT_LIBRARIES} k4a) #TODO let's hope, k4a will never be renamed (their cmake config doesn't define k4a_LIBRARY (or similar))
  endif()
endif()

# zlib
if(USE_IMUTILS GREATER 0)
  if(NOT pvt_FIND_QUIETLY)
    message(STATUS "Some selected PVT modules request zlib as optional dependency")
  endif()

  find_package(ZLIB) # zlib is optional
  if(NOT ZLIB_FOUND)
    message(WARNING "zlib not found!")
  else()
    set(PVT_LIBRARIES ${PVT_LIBRARIES} ${ZLIB_LIBRARIES})
  endif()
endif()

# libconfig++
if(USE_CONFIG GREATER 0)
  if(NOT pvt_FIND_QUIETLY)
    message(STATUS "Some selected PVT modules require libconfig++ as dependency")
  endif()

  find_package(LibConfig++ REQUIRED)
  if(NOT LIBCONFIG++_FOUND)
    message(FATAL_ERROR "libconfig++ not found!")
  endif()

  # Add CURL libraries to PVT library list
  set(PVT_LIBRARIES ${PVT_LIBRARIES} ${LIBCONFIG++_LIBRARY})
  #TODO do we need to add the ${LIBCONFIG++_INCLUDE_DIR}?
endif()

# Find include directory
find_path(PVT_UTILS_INCLUDE_DIR string_utils.h
  HINTS ${POTENTIAL_INCLUDE_DIR}/pvt_utils
  NO_DEFAULT_PATH
)

if(PVT_UTILS_INCLUDE_DIR AND PVT_LIBRARIES)
  set(PVT_FOUND TRUE)
          
  # Set the libraries variable.
  set(PVT_LIBRARIES ${PVT_LIBRARIES})

  # Set include directory to parent dir, s.t. we can include<pvt_utils/xyz.h>.
  get_filename_component(PVT_INCLUDE_DIR "${PVT_UTILS_INCLUDE_DIR}/.." REALPATH)
endif()
  
if(PVT_FOUND)
  if(NOT pvt_FIND_QUIETLY)
    message(STATUS "Found PVT: ${PVT_LIBRARY_DIR} ${PVT_INCLUDE_DIR}")
    message(STATUS "Libs: ${PVT_LIBRARIES}")
  endif()
else()
  if(pvt_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find PVT")
  endif()
endif()
