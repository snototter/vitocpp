cmake_minimum_required(VERSION 2.8)
project(vcp)

##############################################################################
# Version info
##############################################################################
set(VCP_VERSION "1.1.1")

message("[vcp] Configuring C++/Python vision utitility library v${VCP_VERSION}.")
##############################################################################
# Options
##############################################################################
#option(VCP_VERBOSE_TIMING "Verbose timing output, may spam your console" ON)
option(VCP_BUILD_STATIC "Additionally build static libraries." OFF)
option(VCP_BUILD_BEST "Build Best Effort Streaming module." ON)
option(VCP_BUILD_BGM "Build background model module." ON)
option(VCP_BUILD_CONFIG "Build config module." ON)
option(VCP_BUILD_IMUTILS "Build image utilities module." ON)
option(VCP_BUILD_IMVIS "Build visualization module." ON)
option(VCP_BUILD_MATH "Build math utility module." ON)
#option(VCP_BUILD_TRACKING "Build tracking library." ON)
option(VCP_BUILD_UI "Build simple UI utility module." ON)
#option(VCP_BUILD_DOC "Build doxygen documentation" ON)
option(VCP_BUILD_PYTHON "Build Python bindings." ON)
# Everybody was unit-testing:
option(VCP_BUILD_TESTS "Build tests, requires GTest." OFF)


# Logging related
set(VCP_LOG_LEVEL "info" CACHE STRING "Select logging level")
set_property(CACHE VCP_LOG_LEVEL PROPERTY STRINGS debug info warning failure)

option(VCP_LOG_LOCATION "Logged messages include source file location." OFF)

option(VCP_LOG_STACK_TRACE_ON_ERROR "Log stack trace on exception/exit/abort." ON)



##############################################################################
# Set up CMake internals

# Add custom FindPackage scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# Postfix to indicate debug builds (also edit FindXXX.cmake files if you change this!)
set(CMAKE_DEBUG_POSTFIX "d")

# Prefer local install directory at <VCP_ROOT_DIR>/gen
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/gen)

# RPATH for linking
set(CMAKE_INSTALL_RPATH "$ORIGIN")


##############################################################################
# Check build type
##############################################################################
if(NOT CMAKE_BUILD_TYPE)
    message("  * No build type configured, default to Release")
    set(CMAKE_BUILD_TYPE Release)
endif()
#TODO check if needed anymore
# Set flag (in case sub-modules need to know the build type)
string(TOLOWER "${CMAKE_BUILD_TYPE}" LBUILD_TYPE)
if(LBUILD_TYPE MATCHES debug)
  add_definitions(-DDEBUG_BUILD)
  set(VCP_IS_DEBUG_BUILD TRUE)
else()
  set(VCP_IS_DEBUG_BUILD FALSE)
endif()


##############################################################################
# Write current version to disk
string(REGEX MATCHALL "[0-9]" VCP_VERSION_PARTS "${VCP_VERSION}")
list(GET VCP_VERSION_PARTS 0 VCP_VERSION_MAJOR)
list(GET VCP_VERSION_PARTS 1 VCP_VERSION_MINOR)
list(GET VCP_VERSION_PARTS 1 VCP_VERSION_PATCH)

# Keep C++ version header up-to-date
file(WRITE src/cpp/vcp_utils/version.h 
    "#ifndef __VCP_VERSION_H__
#define __VCP_VERSION_H__
#define VCP_VERSION \"${VCP_VERSION}\"
#define VCP_VERSION_MAJOR ${VCP_VERSION_MAJOR}
#define VCP_VERSION_MINOR ${VCP_VERSION_MINOR}
#define VCP_VERSION_PATCH ${VCP_VERSION_PATCH}
#endif // __VCP_VERSION_H__\n")

# Keep version in python 3 bindings up-to-date
file(WRITE src/python3/vcp/version.py
    "__version__ = \"${VCP_VERSION}\"\n")



##############################################################################
# Compiler flags


if(WIN32)
    message(FATAL_ERROR "[${PROJECT_NAME}] TODO Need to specify MSVC-compatible flags, at least for c++11")
else()
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wall -std=c++11 -Wpedantic -Wextra -g -rdynamic")
endif()

# Add OpenMP if available
#find_package(OpenMP)
#if (OPENMP_FOUND)
#    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()


#TODO make separate flag - VCP_WITH_TIMING (TIC_TOC should be available to external projects, too)
#if(VCP_VERBOSE_TIMING)
#  add_definitions(-DPVT_VERBOSE_TIMING)
#endif()


# Logging-related flags
#
# ... pick configured logging level
if(${VCP_LOG_LEVEL} STREQUAL "debug")
    add_definitions(-DVCP_LOG_LEVEL_DEBUG)
    message("  * vcp will log DEBUG, INFO, WARNING and FAILURE messages.")
elseif(${VCP_LOG_LEVEL} STREQUAL "info")
    add_definitions(-DVCP_LOG_LEVEL_INFO)
    message("  * vcp will log INFO, WARNING and FAILURE messages.")
elseif(${VCP_LOG_LEVEL} STREQUAL "warning")
    add_definitions(-DVCP_LOG_LEVEL_WARNING)
    message("  * vcp will only log WARNING and FAILURE messages.")
elseif(${VCP_LOG_LEVEL} STREQUAL "failure")
    add_definitions(-DVCP_LOG_LEVEL_FAILURE)
    message("  * vcp will only log FAILURE messages.")
else()
    message(FATAL_ERROR "[${PROJECT_NAME}] Invalid VCP_LOG_LEVEL value")
endif()
#
# ... should we include the source file location in our logs?
if(VCP_LOG_LOCATION)
  add_definitions(-DVCP_LOG_LOCATION)
endif()
#
# ... should we log the stack trace on unrecoverable errors?
if(VCP_LOG_STACK_TRACE_ON_ERROR)
    add_definitions(-DVCP_LOG_STACK_TRACE_ON_ERROR)
endif()



##############################################################################
# Enable testing if requested
if(VCP_BUILD_TESTS)
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})
    enable_testing()
endif()


##############################################################################
# Set up modules

# Always build utils library
add_subdirectory(src/cpp/vcp_utils)

# Best effort streaming
if(VCP_BUILD_BEST)
    add_subdirectory(src/cpp/vcp_best)
endif()

# Background models
if(VCP_BUILD_BGM)
    add_subdirectory(src/cpp/vcp_bgm)
endif()

# Config module
if(VCP_BUILD_CONFIG)
    add_subdirectory(src/cpp/vcp_config)
endif()

# Imutils utilities
if(VCP_BUILD_IMUTILS)
    add_subdirectory(src/cpp/vcp_imutils)
endif()

# Math utilities
if(VCP_BUILD_MATH)
    add_subdirectory(src/cpp/vcp_math)
endif()

# Tracking library
#if(PVT_BUILD_TRACKING)
#    add_subdirectory(src/pvt_tracking)
#endif()

# User interface module
if(VCP_BUILD_UI)
    add_subdirectory(src/cpp/vcp_ui)
endif()

# Visualizations library
if(VCP_BUILD_IMVIS)
    add_subdirectory(src/cpp/vcp_imvis)
endif()

# Python bindings
if(VCP_BUILD_PYTHON)
    add_subdirectory(src/python3)
endif()


# Add an uninstall target
# See: https://gitlab.kitware.com/cmake/community/wikis/FAQ#can-i-do-make-uninstall-with-cmake
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
        COMMENT "Uninstall files which were created upon 'make install'")
endif()

# Check if Doxygen is installed
find_package(Doxygen QUIET)
if (DOXYGEN_FOUND)
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doc/doxygen.cfg)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # Configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    add_custom_target(doc 
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)
    message("[${PROJECT_NAME}] Use 'make doc' to generate doxygen documentation.\n")
else()
  message("[${PROJECT_NAME}] Won't generate C++ documentation (doxygen is missing)\n")
endif()

