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

