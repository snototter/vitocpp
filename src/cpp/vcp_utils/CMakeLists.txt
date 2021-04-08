project(vcp_utils)

set(LIB_NAME ${PROJECT_NAME})
message("Configuring module ${LIB_NAME}")

# Header files
set(HEADERS 
    vcp_utils.h
    enum_utils.h
    file_utils.h
    vcp_error.h
    vcp_logging.h
    string_utils.h
    circular_buffer.h
    timing_utils.h
    math_utils.h
    sort_utils.h
    stop_watch.h
    version.h
)


# Source files
set(SOURCES
    string_utils.cpp
    file_utils.cpp
    math_utils.cpp
)


# Add the shared library
message("  [+] Building shared ${LIB_NAME} library")
set(LIBS ${LIB_NAME})
add_library(${LIB_NAME} SHARED ${HEADERS} ${SOURCES})


# Add the static library
if(VCP_BUILD_STATIC)
    message("  [+] Building static ${LIB_NAME} library")
    set(LIBS ${LIBS} ${LIB_NAME}_static)
    add_library(${LIB_NAME}_static STATIC ${HEADERS} ${SOURCES})
else()
    message("  [ ] Won't build static ${LIB_NAME} library")
endif()


# Add tests if required
if(VCP_BUILD_TESTS)
    message("  [+] Building ${LIB_NAME} tests\n")
    add_executable(${LIB_NAME}_test
        test_utils.h
        utils_test.cpp
        string_utils_test.cpp
        circular_buffer_test.cpp
        math_utils_test.cpp
    )

    set(EXTRA_TEST_LIBS)
    find_package(OpenCV)
    if(OpenCV_FOUND)
        set(EXTRA_TEST_LIBS ${EXTRA_TEST_LIBS} ${OpenCV_LIBS})
        add_definitions(-DWITH_OPENCV)
    endif()
    # GTest additionally requires pthread
    target_link_libraries(${LIB_NAME}_test
        ${LIB_NAME}
        ${GTEST_LIBRARIES} 
        pthread
        ${EXTRA_TEST_LIBS}
    )
    add_test(${LIB_NAME}_test ${LIB_NAME}_test)
else()
    message("  [ ] Skipping ${LIB_NAME} tests\n")
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

