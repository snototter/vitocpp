project(vcp_config)

set(LIB_NAME ${PROJECT_NAME})

message("Configuring module ${LIB_NAME}")


# Config parsing via libconfig++
find_package(LibConfig++ REQUIRED)
include_directories(${LIBCONFIG++_INCLUDE_DIR})


# Config parsing via JSON
find_package(nlohmannjson REQUIRED)
include_directories(${NLOHMANNJSON_INCLUDE_DIR})


# Header files
set(HEADERS 
    vcp_config.h
    config_params.h
)


# Source files
set(SOURCES
    config_libconfig.cpp
    config_json.cpp
)


# Add the shared library
message("  [+] Building shared ${LIB_NAME} library")
set(LIBS ${LIB_NAME})
add_library(${LIB_NAME} SHARED ${HEADERS} ${SOURCES})
target_link_libraries(${LIB_NAME} 
    vcp_utils
    ${LIBCONFIG++_LIBRARY}
)
target_include_directories(${LIB_NAME}
    PUBLIC
    ${CMAKE_SOURCE_DIR}/src/cpp
)


# Add the static library
if(VCP_BUILD_STATIC)
    message("  [+] Building static ${LIB_NAME} library\n")
    set(LIBS ${LIBS} ${LIB_NAME}_static)
    add_library(${LIB_NAME}_static STATIC ${HEADERS} ${SOURCES})
    target_link_libraries(${LIB_NAME}_static
        vcp_utils
        ${LIBCONFIG++_LIBRARY}
    )
    target_include_directories(${LIB_NAME}_static
        PUBLIC
        ${CMAKE_SOURCE_DIR}/src/cpp
    )
else()
    message("  [ ] Won't build static ${LIB_NAME} library\n")
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

