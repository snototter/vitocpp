cmake_minimum_required(VERSION 2.8)
project(vitocpp-examples)

# Set default build type.
if (NOT CMAKE_BUILD_TYPE)
  message(STATUS "[${PROJECT_NAME}] No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE Release)
endif()

if(WIN32)
    message(FATAL_ERROR "[${PROJECT_NAME}] TODO Need to specify MSVC-compatible flags, at least for c++11")
else()
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wall -std=c++11 -Wpedantic -Wextra -g -rdynamic")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../cmake")


##############################################################################
# Setting up the demos
#
# Note that we (unnecessarily) call find_package(vcp COMPONENTS ...) for each
# demo to load only the minimally required vcp modules.
# Otherwise, you could just use "find_package(vcp)"

# vcp_utils demo
message(STATUS "Setting up utils_demo")
find_package(vcp REQUIRED)
include_directories(${VCP_INCLUDE_DIR})

add_executable(utils_demo
    cpp/utils_demo.cpp
)
target_link_libraries(utils_demo
    ${VCP_LIBRARIES}
)

# vcp_imvis demo
message(STATUS "Setting up visualization_demo")
find_package(vcp REQUIRED COMPONENTS vcp_utils vcp_imvis)
include_directories(${VCP_INCLUDE_DIR})
add_executable(imvis_demo
    cpp/imvis_demo.cpp
)
target_link_libraries(imvis_demo
    ${VCP_LIBRARIES}
)
add_custom_command(TARGET imvis_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/data/peaks.png ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/data/flamingo.jpg ${CMAKE_BINARY_DIR}
)

## kalman demo
#add_executable(kalman_demo
#  kalman_demo.cpp
#)
#target_link_libraries(kalman_demo
#  ${VCP_LIBRARIES}
#)
##
#
# bgm demo
#add_executable(bgm_demo
#  bgm_demo.cpp
#)
#target_link_libraries(bgm_demo
#  ${PVT_LIBRARIES}
#)#
#
# Best Effort Streaming (BESt) demo
message(STATUS "Setting up best_demo")
find_package(vcp REQUIRED COMPONENTS vcp_best vcp_imvis)
include_directories(${VCP_INCLUDE_DIR})
add_executable(best_demo
  cpp/best_demo.cpp
)
target_link_libraries(best_demo
    ${VCP_LIBRARIES}
)
add_custom_command(TARGET best_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/data-best/
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/data/data-best/* ${CMAKE_BINARY_DIR}/data-best/
)

#
# Streaming demo
#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
#add_executable(stream_demo
#  stream_demo.cpp
#)
#target_link_libraries(stream_demo
#  ${VCP_LIBRARIES}
#  ${OpenCV_LIBS}
#${LIVE555_LIBRARIES}
#)
#
# config demo
message(STATUS "Setting up config_demo")
find_package(vcp REQUIRED COMPONENTS vcp_config)
include_directories(${VCP_INCLUDE_DIR})
add_executable(config_demo
    cpp/config_demo.cpp
)
target_link_libraries(config_demo
    ${VCP_LIBRARIES}
)
add_custom_command(TARGET config_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/data/config.cfg ${CMAKE_BINARY_DIR}
)

# UI demo
message(STATUS "Setting up ui_demo")
find_package(vcp REQUIRED COMPONENTS vcp_ui)
include_directories(${VCP_INCLUDE_DIR})
add_executable(ui_demo
    cpp/ui_demo.cpp
)
target_link_libraries(ui_demo
    ${VCP_LIBRARIES}
)
add_custom_command(TARGET ui_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/data/flamingo.jpg ${CMAKE_BINARY_DIR}
)

# math demo
message(STATUS "Setting up math_demo (also imutils::matutils)")
find_package(vcp REQUIRED COMPONENTS vcp_math vcp_imutils)
include_directories(${VCP_INCLUDE_DIR})

add_executable(math_demo
    cpp/math_demo.cpp
)
target_link_libraries(math_demo
    ${VCP_LIBRARIES}
)


# Best Effort Streaming (BESt) demo
message(STATUS "Setting up cairo comparison")
find_package(vcp REQUIRED COMPONENTS vcp_best vcp_imvis vcp_imutils)
include_directories(${VCP_INCLUDE_DIR})

# viren2d requires at least C++14
set(CMAKE_CXX_STANDARD 17)

# We only need the toolbox (no examples, no testing)
set(werkzeugkiste_BUILD_EXAMPLES OFF)
set(werkzeugkiste_BUILD_TESTS OFF)
# We need werkzeugkiste's install targets (since its vector class
# is part of the public viren2d++ interface).
set(werkzeugkiste_INSTALL ON)

# Fetch the library:
include(FetchContent)

FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG master
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE)
set(EIGEN_BUILD_DOC OFF)
# note: To disable eigen tests,
# you should put this code in a add_subdirectory to avoid to change
# BUILD_TESTING for your own project too since variables are directory
# scoped
set(BUILD_TESTING OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
set( OFF)
FetchContent_MakeAvailable(Eigen)

FetchContent_Declare(
    werkzeugkiste
    GIT_REPOSITORY https://github.com/snototter/werkzeugkiste.git
    GIT_TAG main)
FetchContent_MakeAvailable(werkzeugkiste)
# Let CMake download and set up the viren2d dependency
include(FetchContent)
FetchContent_Declare(
  viren2d
  GIT_REPOSITORY https://github.com/snototter/viren2d.git
  GIT_TAG main)
FetchContent_MakeAvailable(viren2d)


add_executable(cairo_comparison
  cpp/cairo_comparison.cpp
)
# Add the viren2d dependency (include paths will be set up automatically)
target_link_libraries(cairo_comparison PRIVATE Eigen3::Eigen werkzeugkiste::timing viren2d++::viren2d++ ${VCP_LIBRARIES})

