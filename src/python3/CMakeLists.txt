cmake_minimum_required(VERSION 3.1)
project(vcp-bindings)

set(PYVCP_LIBRARY_DESTINATION "${CMAKE_SOURCE_DIR}/vcp")

#TODO do we really need 14 for the python bindings, or 11?
# Add C++-14 support
IF(NOT WIN32)
  add_definitions(-std=c++14 -fPIC -Wall -Wpedantic -Wextra)
ENDIF(NOT WIN32)

# We need python3 - use our custom cmake script
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../../cmake")
include("LocatePython")
set(PYTHON_INCLUDE_DIRS ${PYTHON3_INCLUDE_DIR} ${PYTHON3_INCLUDE_DIR2} ${PYTHON3_NUMPY_INCLUDE_DIRS})
set(PYTHON_LIBRARIES ${PYTHON3_LIBRARY})
set(PYTHON_EXECUTABLE ${PYTHON3_EXECUTABLE})
set(PYTHON_PACKAGES_PATH ${PYTHON3_PACKAGES_PATH})
include_directories(${PYTHON_INCLUDE_DIRS})
# Custom postfix to explicitly state we are using py3
set(ARCHIVE_OUTPUT_POSTFIX _py3)

# Get proper extension for python binary shared object on this platform
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import distutils.sysconfig; print(distutils.sysconfig.get_config_var('SO'))"
        RESULT_VARIABLE PYTHON_${PROJECT_NAME}_PY_PROCESS
        OUTPUT_VARIABLE ${PROJECT_NAME}_PY_SUFFIX
        OUTPUT_STRIP_TRAILING_WHITESPACE)

if (MSVC AND NOT PYTHON_DEBUG_LIBRARIES)
    set(PYTHON_INSTALL_CONFIGURATIONS CONFIGURATIONS Release)
else ()
    set(PYTHON_INSTALL_CONFIGURATIONS "")
endif ()

if (WIN32)
    set(PYTHON_INSTALL_ARCHIVE "")
else ()
    set(PYTHON_INSTALL_ARCHIVE ARCHIVE DESTINATION ${PYTHON_PACKAGES_PATH} COMPONENT python)
endif ()

# PyBind11
find_package(pybind11 REQUIRED)
include_directories(${PYBIND11_INCLUDE_DIRS})

# VCP
find_package(vcp REQUIRED COMPONENTS vcp_utils)
include_directories(${VCP_INCLUDE_DIR})
set(VCP_RPATH_ "${VCP_LIBRARY_DIR}")

# Set runtime path so that python knows how to load VCP shared libs
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR})
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}:${VCP_RPATH_}")
message(STATUS "[vcp-bindings] RPATH for VCP libraries: ${VCP_RPATH_}")

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


##################################################################
## Common utils (file, etc.) module
python_add_module(utils
  bindings/utils.cpp
)
target_link_libraries(utils
  ${PYTHON_LIBRARIES}
  ${VCP_LIBRARIES}
)
set_target_properties(utils PROPERTIES
        ARCHIVE_OUTPUT_NAME utils${ARCHIVE_OUTPUT_POSTFIX}
        PREFIX ""
        OUTPUT_NAME utils
        SUFFIX ${${PROJECT_NAME}_PY_SUFFIX}
)
install(TARGETS
  utils
  ${PYTHON_INSTALL_CONFIGURATIONS}
  LIBRARY DESTINATION "${PYVCP_LIBRARY_DESTINATION}")


##################################################################
# Add an uninstall target
# See: https://gitlab.kitware.com/cmake/community/wikis/FAQ#can-i-do-make-uninstall-with-cmake
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()

