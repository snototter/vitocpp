cmake_minimum_required(VERSION 2.8)
project(vcp_ui)

set(LIB_NAME ${PROJECT_NAME})
message("Configuring module ${LIB_NAME}")

# Dependencies
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# Header files
set(HEADERS 
    vcp_ui.h
    point_selection.h
    rect_selection.h
)


# Source files
set(SOURCES
    point_selection.cpp
    rect_selection.cpp
)


# Add the shared library
message("  [+] Building shared ${LIB_NAME} library")
set(LIBS ${LIB_NAME})
add_library(${LIB_NAME} SHARED ${HEADERS} ${SOURCES})
# Uses stuff from the vcp_utils:
target_link_libraries(${LIB_NAME} 
    vcp_utils
    ${OpenCV_LIBS}
)
# We want to include <vcp_utils> instead of using relative paths
target_include_directories(${LIB_NAME}
    PUBLIC
    ${CMAKE_SOURCE_DIR}/src/cpp
)


# Add the static library
if(VCP_BUILD_STATIC)
    message("  [+] Building static ${LIB_NAME} library\n")
    set(LIBS ${LIBS} ${LIB_NAME}_static)
    add_library(${LIB_NAME}_static STATIC ${HEADERS} ${SOURCES})
    target_link_libraries(${LIB_NAME}_static
        vcp_utils
        ${OpenCV_LIBS}
    )
    target_include_directories(${LIB_NAME}_static
        PUBLIC
        ${CMAKE_SOURCE_DIR}/src/cpp
    )
else()
    message("  [ ] Won't build static ${LIB_NAME} library\n")
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


