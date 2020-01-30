cmake_minimum_required(VERSION 2.8)
project(vcp_bgm)

set(LIB_NAME ${PROJECT_NAME})


# Dependencies
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

message("Configuring module ${LIB_NAME}")

# Header files
set(HEADERS 
    background_model.h
#  approx_median_bgm.h
#  blockbased_mean_bgm.h
    normalized_rgb_bgm.h
)


# Source files
set(SOURCES
#  approx_median_bgm.cpp
#  blockbased_mean_bgm.cpp
    normalized_rgb_bgm.cpp
)


# Add the shared library
message("  [+] Building shared ${LIB_NAME} library")
set(LIBS ${LIB_NAME})
add_library(${LIB_NAME} SHARED ${HEADERS} ${SOURCES})
target_link_libraries(${LIB_NAME} 
    vcp_utils
    ${OpenCV_LIBS}
)
target_include_directories(${LIB_NAME}
    PUBLIC
    ${CMAKE_SOURCE_DIR}/src/cpp
)


# Add the static library
if (VCP_BUILD_STATIC)
    message("  [+] Building static ${LIB_NAME} library")
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
else ()
    message("  [ ] Won't build static ${LIB_NAME} library")
endif ()


# Export
install(FILES ${HEADERS}
    DESTINATION include/vcp_bgm
    COMPONENT headers
)
install(TARGETS ${LIBS}
    DESTINATION lib
    EXPORT ${LIB_NAME}
    COMPONENT libraries
)

