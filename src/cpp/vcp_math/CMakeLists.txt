cmake_minimum_required(VERSION 2.8)
project(vcp_math)

set(LIB_NAME ${PROJECT_NAME})
message("Configuring module ${LIB_NAME}")

set(PUBLIC_HEADERS
    geometry2d.h
    geometry3d.h
    common.h
    conversions.h
)
set(PRIVATE_HEADERS
)
set(SOURCES
    geometry2d.cpp
    geometry3d.cpp
    common.cpp
)


# All need OpenCV for matrix handling
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(MATH_EXTERNAL_LIBS
    ${OpenCV_LIBS}
)


# Add the shared library
message("  [+] Building shared ${LIB_NAME} library")
set(LIBS ${LIB_NAME})
add_library(${LIB_NAME} SHARED ${PUBLIC_HEADERS} ${PRIVATE_HEADERS} ${SOURCES})
target_link_libraries(${LIB_NAME} 
    vcp_utils
    ${MATH_EXTERNAL_LIBS}
)
# Add VCP source directory, so we can use vcp_utils
target_include_directories(${LIB_NAME}
    PUBLIC
    ${CMAKE_SOURCE_DIR}/src/cpp
)

#TODO check dependencies of math (utils, maybe imutils?)
# Add the static library
if(VCP_BUILD_STATIC)
    message("  [+] Building static ${LIB_NAME} library")
    set(LIBS ${LIBS} ${LIB_NAME}_static)
    add_library(${LIB_NAME}_static STATIC ${PUBLIC_HEADERS} ${PRIVATE_HEADERS} ${SOURCES})
    target_link_libraries(${LIB_NAME}_static
      vcp_utils
      ${MATH_EXTERNAL_LIBS}
    )
    target_include_directories(${LIB_NAME}_static
      PUBLIC
      ${CMAKE_SOURCE_DIR}/src/cpp
    )
else()
    message("  [ ] Won't build static ${LIB_NAME} library")
endif()


# Add tests if required
if (VCP_BUILD_TESTS)
    # See https://www.eriksmistad.no/getting-started-with-google-test-on-ubuntu/
    message("  [+] Building ${LIB_NAME} tests")
    add_executable(${LIB_NAME}_test
      math_test.cpp
      common_test.cpp
    )

  # GTest additionally requires pthread
    target_link_libraries(${LIB_NAME}_test
      ${LIB_NAME}
      ${GTEST_LIBRARIES}
      pthread
    )
    add_test(${LIB_NAME}_test ${LIB_NAME}_test)
else()
    message("  [ ] Skipping ${LIB_NAME} tests")
endif()


# Export (only public headers and lib)
install(FILES ${PUBLIC_HEADERS}
  DESTINATION include/${PROJECT_NAME}
  COMPONENT headers
)
install(TARGETS ${LIBS}
  DESTINATION lib
  EXPORT ${LIB_NAME}
  COMPONENT libraries
)

