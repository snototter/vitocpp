# Adapted from:
# Module: Findpybind11.cmake
# Author: Bastian Rieck <bastian.rieck@iwr.uni-heidelberg.de>
#
# https://github.com/Pseudomanifold/cmake-cpp-pybind11/blob/master/cmake/Modules/Findpybind11.cmake
#
# CMake find module for pybind11.

INCLUDE( FindPackageHandleStandardArgs )

MACRO( SET_IF_EMPTY variable value )
  IF( "${${variable}}" STREQUAL "" )
    SET( ${variable} ${value} )
  ENDIF()
ENDMACRO()

SET_IF_EMPTY( PYBIND11_DIR "$ENV{PYBIND11_DIR}" )
get_filename_component(VCP_CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)
SET_IF_EMPTY( PYBIND11_DIR "${VCP_CMAKE_MODULE_PATH}/../third-party/pybind11-2.4.3/install/include" )


FIND_PATH(
  PYBIND11_INCLUDE_DIR
    pybind11/pybind11.h
  HINTS
    ${PYBIND11_DIR}
)

FIND_PACKAGE_HANDLE_STANDARD_ARGS( PYBIND11 DEFAULT_MSG
  PYBIND11_INCLUDE_DIR
)

IF( PYBIND11_FOUND )
  SET( PYBIND11_INCLUDE_DIRS ${PYBIND11_INCLUDE_DIR} )

  MARK_AS_ADVANCED(
    PYBIND11_INCLUDE_DIR
    PYBIND11_DIR
  )
ELSE()
  SET( PYBIND11_DIR "" CACHE STRING
    "An optional hint to a pybind11 directory"
  )
  IF(pybind11_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find pybind11")
  ENDIF()
ENDIF()
