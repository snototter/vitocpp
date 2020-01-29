# - Tries to find LIVE555 within VITO's "external" subfolder
# Once done this will define
#  
#  Live555_FOUND
#  Live555_INCLUDE_DIRS
#  Live555_LIBRARIES
#
# NOTE: The linking order of libraries is FREAKIN' important!

if(WIN32)
  string(REPLACE "\\" "/" LIVE555_ROOT_DIR $ENV{LIVE555_ROOT_DIR})
else(WIN32)
  set(LIVE555_ROOT_DIR $ENV{LIVE555_ROOT_DIR})
  if(NOT EXISTS ${LIVE555_ROOT_DIR})
    get_filename_component(LIVE555_ROOT_DIR "../external/live" REALPATH BASE_DIR "${CMAKE_CURRENT_LIST_DIR}")
    #message(Status "LIVE555_ROOT_DIR not set, looking for library in: ${LIVE555_ROOT_DIR}")
  endif()
endif(WIN32)

set(LIVE555_INCLUDE_DIRS
  "${LIVE555_ROOT_DIR}/BasicUsageEnvironment/include"
  "${LIVE555_ROOT_DIR}/UsageEnvironment/include" 
  "${LIVE555_ROOT_DIR}/groupsock/include" 
  "${LIVE555_ROOT_DIR}/liveMedia/include" 
)
set(LIVE555_LIBRARIES 
  "${LIVE555_ROOT_DIR}/liveMedia/libliveMedia.a"
  "${LIVE555_ROOT_DIR}/groupsock/libgroupsock.a"
  "${LIVE555_ROOT_DIR}/BasicUsageEnvironment/libBasicUsageEnvironment.a"
  "${LIVE555_ROOT_DIR}/UsageEnvironment/libUsageEnvironment.a"
)

# Check if all include dirs and libs exist
set(LIVE555_FOUND true)
foreach(INC_DIR ${LIVE555_INCLUDE_DIRS})
  if(NOT EXISTS ${INC_DIR})
    set(LIVE555_FOUND false)
    # TODO check if required, etc.
    message(FATAL_ERROR "Cannot find live555 include dir: " ${INC_DIR})
  endif()
endforeach()

foreach(LIB_ ${LIVE555_LIBRARIES})
  if(NOT EXISTS ${LIB_})
    set(LIVE555_FOUND false)
    # TODO check if required, etc.
    message(FATAL_ERROR "Cannot find live555 library: " ${LIB_})
  endif()
endforeach()


