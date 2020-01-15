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

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} $ENV{PVT_ROOT_DIR}/cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(vcp REQUIRED COMPONENTS vcp_utils)
include_directories(${VCP_INCLUDE_DIR})

# vcp_utils demo
add_executable(utils_demo
  utils_demo.cpp
)
target_link_libraries(utils_demo
  ${VCP_LIBRARIES}
)

## pvt_visualization demo
#add_executable(visualization_demo
#  visualization_demo.cpp
#)
#target_link_libraries(visualization_demo
#  ${PVT_LIBRARIES}
#)
#add_custom_command(TARGET visualization_demo POST_BUILD
#  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/data/peaks.png ${CMAKE_BINARY_DIR}
#)
#
## kalman demo
#add_executable(kalman_demo
#  kalman_demo.cpp
#)
#target_link_libraries(kalman_demo
#  ${PVT_LIBRARIES}
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
## ip cam capture (icc) demo
#find_package(LIVE555 REQUIRED)
#  include_directories(${LIVE555_INCLUDE_DIRS})
#add_executable(icc_demo
#  icc_demo.cpp
#)
#target_link_libraries(icc_demo
#  ${PVT_LIBRARIES}
#${LIVE555_LIBRARIES}
#)
#add_custom_command(TARGET icc_demo POST_BUILD
#  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/data/*.png ${CMAKE_BINARY_DIR}
#  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/data/*.cfg ${CMAKE_BINARY_DIR}
#  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/data/*.sdp ${CMAKE_BINARY_DIR}
#)
#
# Streaming demo
#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
#add_executable(stream_demo
#  stream_demo.cpp
#)
#target_link_libraries(stream_demo
#  ${PVT_LIBRARIES}
#  ${OpenCV_LIBS}
#${LIVE555_LIBRARIES}
#)
#
# config access demo
#add_executable(config_demo
#  config_demo.cpp
#)
#target_link_libraries(config_demo
#  ${PVT_LIBRARIES}
#)
#add_custom_command(TARGET config_demo POST_BUILD
 # COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/data/config.cfg ${CMAKE_BINARY_DIR}
#)

# config access demo
#add_executable(ui_demo
#  ui_demo.cpp
#)
#target_link_libraries(ui_demo
#  ${PVT_LIBRARIES}
#)
#add_custom_command(TARGET ui_demo POST_BUILD
#  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/data/lena.jpg ${CMAKE_BINARY_DIR}
#)

# math demo
#add_executable(math_demo
#  math_demo.cpp
#)
#target_link_libraries(math_demo
#  ${PVT_LIBRARIES}
#)

