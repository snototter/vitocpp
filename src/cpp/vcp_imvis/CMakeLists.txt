cmake_minimum_required(VERSION 2.8)
project(vcp_imvis)

set(LIB_NAME ${PROJECT_NAME})
message("Configuring module ${LIB_NAME}")

# Dependencies
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# Header files
set(HEADERS 
    pseudocolor.h
    anaglyph.h
    collage.h
    drawing.h
    trajectories.h
)


# Source files
set(SOURCES
    pseudocolor.cpp
    anaglyph.cpp
    collage.cpp
    drawing.cpp
    trajectories.cpp
)


# Add the shared library
message("  [+] Building shared ${LIB_NAME} library")
set(LIBS ${LIB_NAME})
add_library(${LIB_NAME} SHARED ${HEADERS} ${SOURCES})
target_link_libraries(${LIB_NAME} 
    vcp_utils
    vcp_imutils
    vcp_math
    ${OpenCV_LIBS}
)
target_include_directories(${LIB_NAME}
  PUBLIC
  ${CMAKE_SOURCE_DIR}/src/cpp
)


# Add the static library
if(VCP_BUILD_STATIC)
    message("  [+] Building static ${LIB_NAME} library")
    set(LIBS ${LIBS} ${LIB_NAME}_static)
    add_library(${LIB_NAME}_static STATIC ${HEADERS} ${SOURCES})
    target_link_libraries(${LIB_NAME}_static
      vcp_utils
      vcp_imutils
      vcp_math
      ${OpenCV_LIBS}
    )
    target_include_directories(${LIB_NAME}_static
      PUBLIC
      ${CMAKE_SOURCE_DIR}/src/cpp
    )
else()
    message("  [ ] Won't build static ${LIB_NAME} library")
endif()


# Export
install(FILES ${HEADERS}
  DESTINATION include/${PROJECT_NAME}
  COMPONENT headers
)
install(TARGETS ${LIBS}
  DESTINATION lib
  EXPORT ${LIB_NAME}
  COMPONENT libraries
)
