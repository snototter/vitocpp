cmake_minimum_required(VERSION 3.1)
project(vcp-bindings)

#TODO move to utils
#TODO check https://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
foreach(i ${CMAKE_CXX_COMPILE_FEATURES})
    message("${i}")
endforeach()

set(PYVCP_LIBRARY_DESTINATION "${CMAKE_SOURCE_DIR}/../../gen/vcp")

#TODO https://stackoverflow.com/questions/16212538/cmake-depending-on-another-project
#TODO do we really need 14 for the python bindings, or 11?
#TODO install to VCP_ROOT_DIR/gen/python/vcp
# Add C++-14 support
IF(NOT WIN32)
    add_definitions(-std=c++11 -fPIC -Wall -Wpedantic -Wextra)
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

# VCP - prepare include directory and RPATH
find_package(vcp REQUIRED)
include_directories(${VCP_INCLUDE_DIR})
set(VCP_RPATH_ "${VCP_LIBRARY_DIR}")

# Set runtime path so that python knows how to load VCP shared libs
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR})
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}:${VCP_RPATH_}")
message(STATUS "[vcp-bindings] RPATH for python vcp libraries: ${VCP_RPATH_}")

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


##################################################################
## Common utils (file, etc.) module
message(STATUS "[vcp-bindings] Configuring vcp.utils")
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
## Config module - needed for configuration of BESt
python_add_module(config
    bindings/config.cpp
    bindings/config.h
)
target_link_libraries(config
    ${PYTHON_LIBRARIES}
    ${VCP_LIBRARIES}
)
set_target_properties(config PROPERTIES
    ARCHIVE_OUTPUT_NAME config${ARCHIVE_OUTPUT_POSTFIX}
    PREFIX ""
    OUTPUT_NAME config
    SUFFIX ${${PROJECT_NAME}_PY_SUFFIX}
)
install(TARGETS
    config
    ${PYTHON_INSTALL_CONFIGURATIONS}
    LIBRARY DESTINATION "${PYVCP_LIBRARY_DESTINATION}")


##################################################################
## Math 2D
message(STATUS "[vcp-bindings] Configuring vcp.math2d")
python_add_module(math2d
    bindings/math2d.cpp
    bindings/conversion/vcp_conversion.h
    bindings/conversion/vcp_conversion.cpp
    bindings/conversion/np_cv_conversion.h
    bindings/conversion/np_cv_conversion.cpp
    bindings/conversion/cv_core_conversion.h
    bindings/conversion/cv_core_conversion.cpp
)
target_link_libraries(math2d
    ${PYTHON_LIBRARIES}
    ${VCP_LIBRARIES}
)
set_target_properties(math2d PROPERTIES
    ARCHIVE_OUTPUT_NAME math2d${ARCHIVE_OUTPUT_POSTFIX}
    PREFIX ""
    OUTPUT_NAME math2d
    SUFFIX ${${PROJECT_NAME}_PY_SUFFIX}
)
install(TARGETS
    math2d
    ${PYTHON_INSTALL_CONFIGURATIONS}
    LIBRARY DESTINATION "${PYVCP_LIBRARY_DESTINATION}")


##################################################################
## Math 3D
message(STATUS "[vcp-bindings] Configuring vcp.math3d")
python_add_module(math3d
    bindings/math3d.cpp
    bindings/conversion/vcp_conversion.h
    bindings/conversion/vcp_conversion.cpp
    bindings/conversion/np_cv_conversion.h
    bindings/conversion/np_cv_conversion.cpp
    bindings/conversion/cv_core_conversion.h
    bindings/conversion/cv_core_conversion.cpp
)
target_link_libraries(math3d
    ${PYTHON_LIBRARIES}
    ${VCP_LIBRARIES}
)
set_target_properties(math3d PROPERTIES
    ARCHIVE_OUTPUT_NAME math3d${ARCHIVE_OUTPUT_POSTFIX}  # prevent name conflict for python2/3 outputs
    PREFIX ""
    OUTPUT_NAME math3d
    SUFFIX ${${PROJECT_NAME}_PY_SUFFIX}
)
install(TARGETS
    math3d
    ${PYTHON_INSTALL_CONFIGURATIONS}
    LIBRARY DESTINATION "${PYVCP_LIBRARY_DESTINATION}")


##################################################################
## BGM utilities module
python_add_module(bgm
    bindings/bgm.cpp
    bindings/conversion/np_cv_conversion.h
    bindings/conversion/np_cv_conversion.cpp
    bindings/conversion/cv_core_conversion.h
    bindings/conversion/cv_core_conversion.cpp
)
target_link_libraries(bgm
    ${PYTHON_LIBRARIES}
    ${VCP_LIBRARIES}
)
set_target_properties(bgm PROPERTIES
    ARCHIVE_OUTPUT_NAME bgm${ARCHIVE_OUTPUT_POSTFIX}
    PREFIX ""
    OUTPUT_NAME bgm
    SUFFIX ${${PROJECT_NAME}_PY_SUFFIX}
)
install(TARGETS
    bgm
    ${PYTHON_INSTALL_CONFIGURATIONS}
    LIBRARY DESTINATION "${PYVCP_LIBRARY_DESTINATION}")

##################################################################
## Image manipulation (imutils)
message(STATUS "[vcp-bindings] Configuring vcp.imutils_cpp")
python_add_module(imutils_cpp
    bindings/imutils_cpp.cpp
    bindings/conversion/np_cv_conversion.h
    bindings/conversion/np_cv_conversion.cpp
    bindings/conversion/cv_core_conversion.h
    bindings/conversion/cv_core_conversion.cpp
)
target_link_libraries(imutils_cpp
    ${PYTHON_LIBRARIES}
    ${VCP_LIBRARIES}
)
set_target_properties(imutils_cpp PROPERTIES
    ARCHIVE_OUTPUT_NAME imutils_cpp${ARCHIVE_OUTPUT_POSTFIX}
    PREFIX ""
    OUTPUT_NAME imutils_cpp
    SUFFIX ${${PROJECT_NAME}_PY_SUFFIX}
)
install(TARGETS
    imutils_cpp
    ${PYTHON_INSTALL_CONFIGURATIONS}
    LIBRARY DESTINATION "${PYVCP_LIBRARY_DESTINATION}")



##################################################################
## Visualization utils
message(STATUS "[vcp-bindings] Configuring vcp.imvis_cpp")
python_add_module(imvis_cpp
    bindings/imvis_cpp.cpp
    bindings/conversion/vcp_conversion.h
    bindings/conversion/vcp_conversion.cpp
    bindings/conversion/np_cv_conversion.h
    bindings/conversion/np_cv_conversion.cpp
    bindings/conversion/cv_core_conversion.h
    bindings/conversion/cv_core_conversion.cpp
)
target_link_libraries(imvis_cpp
    ${PYTHON_LIBRARIES}
    ${VCP_LIBRARIES}
)
set_target_properties(imvis_cpp PROPERTIES
    ARCHIVE_OUTPUT_NAME imvis_cpp${ARCHIVE_OUTPUT_POSTFIX}
    PREFIX ""
    OUTPUT_NAME imvis_cpp
    SUFFIX ${${PROJECT_NAME}_PY_SUFFIX}
)
install(TARGETS
    imvis_cpp
    ${PYTHON_INSTALL_CONFIGURATIONS}
    LIBRARY DESTINATION "${PYVCP_LIBRARY_DESTINATION}")


##################################################################
## Simple UI inputs
message(STATUS "[vcp-bindings] Configuring vcp.ui")
python_add_module(ui_basics
    bindings/ui_basics.cpp
    bindings/conversion/np_cv_conversion.h
    bindings/conversion/np_cv_conversion.cpp
    bindings/conversion/cv_core_conversion.h
    bindings/conversion/cv_core_conversion.cpp
)
target_link_libraries(ui_basics
    ${PYTHON_LIBRARIES}
    ${VCP_LIBRARIES}
)
set_target_properties(ui_basics PROPERTIES
    ARCHIVE_OUTPUT_NAME ui_basics${ARCHIVE_OUTPUT_POSTFIX}
    PREFIX ""
    OUTPUT_NAME ui_basics
    SUFFIX ${${PROJECT_NAME}_PY_SUFFIX}
)
install(TARGETS
    ui_basics
    ${PYTHON_INSTALL_CONFIGURATIONS}
    LIBRARY DESTINATION "${PYVCP_LIBRARY_DESTINATION}")


##################################################################
## Python files
message(STATUS "[vcp-bindings] Configuring install target for python files")
install(DIRECTORY vcp/
    DESTINATION ${PYVCP_LIBRARY_DESTINATION}
    FILES_MATCHING PATTERN "*.py")


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

