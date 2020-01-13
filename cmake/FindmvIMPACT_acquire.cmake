# - Try to find "mvIMPACT acquire" SDK
# Once done this will define
#  mvIMPACT_acquire_FOUND 
#  mvIMPACT_acquire_INCLUDE_DIR
#  mvIMPACT_acquire_LIBS
# You must ensure that mvIMPACT_acquire installation properly set up your MVIMPACT_ACQUIRE_DIR environmental variable!

#TODO this is a quick-hack and only suitable for 64bit linux!
# See https://cmake.org/pipermail/cmake/2011-June/045086.html for how to detect 64 bits (check the size of a void ptr)
if((NOT UNIX) OR (NOT CMAKE_SIZEOF_VOID_P EQUAL 8))
  message(FATAL_ERROR "The FindmvIMPACT_acquire.cmake is only suitable for 64 bit Linux systems!")
endif()

if(WIN32)
  string(REPLACE "\\" "/" MVIA_ROOT $ENV{MVIMPACT_ACQUIRE_DIR})
else(WIN32)
  set(MVIA_ROOT $ENV{MVIMPACT_ACQUIRE_DIR})
endif(WIN32)

if (NOT MVIA_ROOT)
  if(mvIMPACT_acquire_FIND_REQUIRED)
    message(FATAL_ERROR "Environmental variable MVIMPACT_ACQUIRE_DIR not set!")
  endif()
endif()

# Include directory (starts at install root - what a great decision...)
set(mvIMPACT_acquire_INCLUDE_DIR ${MVIA_ROOT})

# Search for the MV libraries
set(_mvia_libdir ${MVIA_ROOT}/lib/x86_64)
set(_mvia_libs mvDeviceManager mvPropHandling)
set(_mvia_all_libs_found TRUE)
foreach(_mvia_lib ${_mvia_libs})
  unset(_lib CACHE)
  find_library(_lib NAMES ${_mvia_lib}
    HINTS ${_mvia_libdir}
#    NO_DEFAULT_PATH
  )
  if (NOT _lib)
    if(NOT mvIMPACT_acquire_FIND_QUIETLY)
      message(STATUS "Cannot find MatrixVision library ${_mvia_lib}")
    endif()
    set(_mvia_all_libs_found FALSE)
  else()
    set(mvIMPACT_acquire_LIBS ${mvIMPACT_acquire_LIBS} ${_lib})
  endif()
endforeach()

if(mvIMPACT_acquire_INCLUDE_DIR AND _mvia_all_libs_found)
  set(mvIMPACT_acquire_FOUND TRUE)
endif()
  
if(mvIMPACT_acquire_FOUND)
  if(NOT mvIMPACT_acquire_FIND_QUIETLY)
    message(STATUS "Found mvIMPACT_acquire at ${mvIMPACT_acquire_INCLUDE_DIR}")
    message(STATUS "Libs: ${mvIMPACT_acquire_LIBS}")
  endif()
else()
  if(mvIMPACT_acquire_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find mvIMPACT_acquire")
  endif()
endif()
