cmake_minimum_required(VERSION 2.8)
project(vcp_imutils)

set(LIB_NAME ${PROJECT_NAME})

message("Configuring module ${LIB_NAME}")

set(PUBLIC_HEADERS
  opencv_compatibility.h
  imabstraction.h
  imutils.h
  matutils.h
)
set(PRIVATE_HEADERS
)
set(SOURCES
  imabstraction.cpp
  imutils.cpp
  matutils.cpp
)

# We need OpenCV for matrix handling
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
set(IMUTILS_EXTERNAL_LIBS
  ${OpenCV_LIBS}
)


# We (optionally) need zlib to dump/load compressed raw images
find_package(ZLIB)
if (ZLIB_FOUND)
  include_directories(${ZLIB_INCLUDE_DIRS})
  set(IMUTILS_EXTERNAL_LIBS ${IMUTILS_EXTERNAL_LIBS} ${ZLIB_LIBRARIES})
  add_definitions(-DVCP_WITH_ZLIB)
  message("  [+] Enable I/O for compressed raw matrices using zlib")
else()
  message("  [ ] Disable I/O for compressed raw matrices (zlib is missing)")
endif()


# Add the shared library
message("  [+] Building shared ${LIB_NAME} library")
set(LIBS ${LIB_NAME})
add_library(${LIB_NAME} SHARED ${PUBLIC_HEADERS} ${PRIVATE_HEADERS} ${SOURCES})
target_link_libraries(${LIB_NAME} 
  vcp_utils
  vcp_math
  ${IMUTILS_EXTERNAL_LIBS}
)
target_include_directories(${LIB_NAME}
  PUBLIC
  ${CMAKE_SOURCE_DIR}/src/cpp
)


# Add the static library
if(VCP_BUILD_STATIC)
  message("  [+] Building static ${LIB_NAME} library")
  set(LIBS ${LIBS} ${LIB_NAME}_static)
  add_library(${LIB_NAME}_static STATIC ${PUBLIC_HEADERS} ${PRIVATE_HEADERS} ${SOURCES})
  target_link_libraries(${LIB_NAME}_static 
    vcp_utils
    vcp_math
    ${IMUTILS_EXTERNAL_LIBS}
  )
  target_include_directories(${LIB_NAME}_static
    PUBLIC
    ${CMAKE_SOURCE_DIR}/src/cpp
  )
else()
  message("  [ ] Won't build static ${LIB_NAME} library")
endif()


# Export (only public headers)
install(FILES ${PUBLIC_HEADERS}
  DESTINATION include/vcp_imutils
  COMPONENT headers
)
install(TARGETS ${LIBS}
  DESTINATION lib
  EXPORT ${LIB_NAME}
  COMPONENT libraries
)

