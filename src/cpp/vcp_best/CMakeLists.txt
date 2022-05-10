project(vcp_best)

set(LIB_NAME ${PROJECT_NAME})
message("Configuring module ${LIB_NAME}")


option(VCP_BEST_WITH_IPCAM "Build HTTP/RTSP streaming module (requires CURL, live555 and ffmpeg)." ON)
#option(VCP_BEST_WITH_MATRIXVISION "Build MatrixVision streaming module (requires mvIMPACT_acquire SDK)." OFF)
option(VCP_BEST_WITH_REALSENSE2 "Build Intel RealSense streaming module (requires librealsense2)." OFF)
option(VCP_BEST_WITH_K4A "Build Kinect 4 Azure streaming module (requires libk4a)." OFF)
option(VCP_BEST_WITH_K4A_MJPG "K4A sends color as MJPG if ON (slow), otherwise BGRA32 (high bandwidth)." OFF)
option(VCP_BEST_WITH_ZED "Build ZED streaming module (requires StereoLabs ZED SDK)." OFF)
option(VCP_BEST_DEBUG_FRAMERATE "If enabled, sinks debug their frame rate (rather verbose)" OFF)

if(VCP_BEST_DEBUG_FRAMERATE)
    add_definitions(-DVCP_BEST_DEBUG_FRAMERATE)
endif()

# HTTP Streaming Files
set(PUBLIC_HTTP_STREAM_HEADERS
    http_mjpeg_sink.h
    curl_file_handling.h
)
set(PRIVATE_HTTP_STREAM_HEADERS
)
set(HTTP_STREAM_SOURCES
    http_mjpeg_sink.cpp
    curl_file_handling.cpp
)


# RTSP Streaming Files
set(PUBLIC_RTSP_STREAM_HEADERS
    rtsp_sink.h
)
set(PRIVATE_RTSP_STREAM_HEADERS
    rtsp_media_sink.h
    rtsp_client.h
)
set(RTSP_STREAM_SOURCES
    rtsp_sink.cpp
    rtsp_media_sink.cpp
    rtsp_client.cpp
)


# MatrixVision streaming files
set(PUBLIC_MATRIXVISION_STREAM_HEADERS
    matrixvision_sink.h
    capture_matrixvision.h
)
set(PRIVATE_MATRIXVISION_STREAM_HEADERS
    matrixvision_opencv.h
    matrixvision_utils.h
)
set(MATRIXVISION_STREAM_SOURCES
    matrixvision_sink.cpp
    matrixvision_opencv.cpp
    capture_matrixvision.cpp
)


# Realsense2 streaming files
set(PUBLIC_REALSENSE2_STREAM_HEADERS
    realsense2_sink.h
)
set(PRIVATE_REALSENSE2_STREAM_HEADERS)
set(REALSENSE2_STREAM_SOURCES
    realsense2_sink.cpp
)


# K4A Kinect Azure streaming files
set(PUBLIC_K4A_STREAM_HEADERS
    k4a_sink.h
)
set(PRIVATE_K4A_STREAM_HEADERS)
set(K4A_STREAM_SOURCES
    k4a_sink.cpp
)


# StereoLabs ZED streaming files
set(PUBLIC_ZED_STREAM_HEADERS
    zed_sink.h
)
set(PRIVATE_ZED_STREAM_HEADERS)
set(ZED_STREAM_SOURCES
    zed_sink.cpp
)


# Common header files - will be extended below for optional devices
set(PUBLIC_HEADERS
    sink.h
    sink_buffer.h
    file_sink.h
    webcam_sink.h
    capture.h
    calibration.h
    liveview.h
    rgbd.h
)
set(PRIVATE_HEADERS)

# Common source files - will be extended below for optional devices
set(SOURCES
    sink.cpp
    file_sink.cpp
    webcam_sink.cpp
    capture.cpp
    calibration.cpp
    liveview.cpp
    rgbd.cpp
)

# All need OpenCV for image conversion
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(ICC_EXTERNAL_LIBS
    ${OpenCV_LIBS}
)



if(VCP_BEST_WITH_IPCAM)
    add_definitions(-DVCP_BEST_WITH_IPCAM -DVCP_BEST_WITH_IPCAM_HTTP -DVCP_BEST_WITH_IPCAM_RTSP)

    set(PUBLIC_HEADERS ${PUBLIC_HEADERS} ipcam_sink.h)
    set(SOURCES ${SOURCES} ipcam_sink.cpp)
    #################################################
    # HTTP streaming
    # Dependencies
    find_package(CURL REQUIRED)
    include_directories(${CURL_INCLUDE_DIRS})
    set(ICC_EXTERNAL_LIBS ${ICC_EXTERNAL_LIBS} ${CURL_LIBRARIES})

    # Source
    set(PUBLIC_HEADERS ${PUBLIC_HEADERS} ${PUBLIC_HTTP_STREAM_HEADERS})
    set(PRIVATE_HEADERS ${PRIVATE_HEADERS} ${PRIVATE_HTTP_STREAM_HEADERS})
    set(SOURCES ${SOURCES} ${HTTP_STREAM_SOURCES})

    #################################################
    # RTSP streaming
    ## FFMPEG - needed for h264 decoding
    find_package(FFmpeg REQUIRED)
    include_directories(${FFMPEG_INCLUDE_DIR})

    ## Live555 - needed for RTSP streaming
    find_package(LIVE555 REQUIRED)
    include_directories(${LIVE555_INCLUDE_DIRS})

    ## SSL needed for streaming
    find_package(OpenSSL REQUIRED)
    include_directories(${OPENSSL_INCLUDE_DIR})

    set(ICC_EXTERNAL_LIBS ${ICC_EXTERNAL_LIBS} ${LIVE555_LIBRARIES} ${FFMPEG_LIBRARIES} ${OPENSSL_LIBRARIES})

    # Source
    set(PUBLIC_HEADERS ${PUBLIC_HEADERS} ${PUBLIC_RTSP_STREAM_HEADERS})
    set(PRIVATE_HEADERS ${PRIVATE_HEADERS} ${PRIVATE_RTSP_STREAM_HEADERS})
    set(SOURCES ${SOURCES} ${RTSP_STREAM_SOURCES})

    message("  [+] Enable HTTP/RTSP streaming")
else()
    message("  [ ] Disable HTTP/RSTP streaming")
endif()


if (VCP_BEST_WITH_MATRIXVISION)
  ## mvIMPACT_acquire to access mvBlueFox3
  find_package(mvIMPACT_acquire REQUIRED QUIET)
  include_directories(${mvIMPACT_acquire_INCLUDE_DIR})

  add_definitions(-DWITH_MATRIXVISION)

  set(ICC_EXTERNAL_LIBS ${ICC_EXTERNAL_LIBS} ${mvIMPACT_acquire_LIBS})

  # Source
  set(PUBLIC_HEADERS ${PUBLIC_HEADERS} ${PUBLIC_MATRIXVISION_STREAM_HEADERS})
  set(PRIVATE_HEADERS ${PRIVATE_HEADERS} ${PRIVATE_MATRIXVISION_STREAM_HEADERS})
  set(SOURCES ${SOURCES} ${MATRIXVISION_STREAM_SOURCES})

  message("  [+] Enable MatrixVision cameras")
else()
  message("  [ ] Disable MatrixVision cameras")
endif()


if (VCP_BEST_WITH_REALSENSE2)
    add_definitions(-DVCP_BEST_WITH_REALSENSE2)
    find_package(realsense2 REQUIRED)
    include_directories(${realsense_INCLUDE_DIR})

    set(ICC_EXTERNAL_LIBS ${ICC_EXTERNAL_LIBS} ${realsense2_LIBRARY})
    set(PUBLIC_HEADERS ${PUBLIC_HEADERS} ${PUBLIC_REALSENSE2_STREAM_HEADERS})
    set(PRIVATE_HEADERS ${PRIVATE_HEADERS} ${PRIVATE_REALSENSE2_STREAM_HEADERS})
    set(SOURCES ${SOURCES} ${REALSENSE2_STREAM_SOURCES})

    message("  [+] Enable Intel RealSense sensors")
else()
    message("  [ ] Disable Intel RealSense sensors")
endif()

if (VCP_BEST_WITH_K4A)
    find_package(k4a REQUIRED)

    add_definitions(-DVCP_BEST_WITH_K4A)

    set(ICC_EXTERNAL_LIBS ${ICC_EXTERNAL_LIBS} k4a)
    set(PUBLIC_HEADERS ${PUBLIC_HEADERS} ${PUBLIC_K4A_STREAM_HEADERS})
    set(PRIVATE_HEADERS ${PRIVATE_HEADERS} ${PRIVATE_K4A_STREAM_HEADERS})
    set(SOURCES ${SOURCES} ${K4A_STREAM_SOURCES})

    message("  [+] Enable Azure Kinect sensors")

    if (VCP_BEST_WITH_K4A_MJPG)
        add_definitions(-DVCP_BEST_WITH_K4A_MJPG)
        message("      * Using JPG encoded K4A color stream")
    else()
        message("      * Using raw BGRA32 K4A color stream")
    endif()
else()
    message("  [ ] Disable Azure Kinect sensors")
endif()


if (VCP_BEST_WITH_ZED)
    option(ZED_DIR "ZED SDK path." "/usr/local/zed/")
    find_package(ZED 3 REQUIRED)
    include_directories(${ZED_INCLUDE_DIRS})
    link_directories(${ZED_LIBRARY_DIR})

    # On ZED SDK 3.0.2 the ZED_CUDA_VERSION is empty
    find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
    include_directories(${CUDA_INCLUDE_DIRS})
    link_directories(${CUDA_LIBRARY_DIRS})

    # According to the StereoLabs examples, we need to include pthreads
    # on non-windows platforms, e.g. see https://www.stereolabs.com/blog/hello-world/
    if(WIN32)
        set(ZED_EXTRA_LIBS)
    else()
        set(ZED_EXTRA_LIBS "pthread")
    endif()
 
 
    add_definitions(-DVCP_BEST_WITH_ZED)
    set(ICC_EXTERNAL_LIBS ${ICC_EXTERNAL_LIBS} ${ZED_LIBRARIES} ${CUDA_DEP_LIBRARIES_ZED} ${ZED_EXTRA_LIBS})
    set(PUBLIC_HEADERS ${PUBLIC_HEADERS} ${PUBLIC_ZED_STREAM_HEADERS})
    set(PRIVATE_HEADERS ${PRIVATE_HEADERS} ${PRIVATE_ZED_STREAM_HEADERS})
    set(SOURCES ${SOURCES} ${ZED_STREAM_SOURCES})
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${ZED_LIBRARY_DIR}")

    message("  [+] Enable ZED stereo sensors")
else()
    message("  [ ] Disable ZED stereo sensors")
endif()


# Add the shared library
message("  [+] Building shared ${LIB_NAME} library")
set(LIBS ${LIB_NAME})
add_library(${LIB_NAME} SHARED ${PUBLIC_HEADERS} ${PRIVATE_HEADERS} ${SOURCES})
target_link_libraries(${LIB_NAME} 
  vcp_utils
  vcp_imutils
  vcp_math
  vcp_config
  ${ICC_EXTERNAL_LIBS}
)
target_include_directories(${LIB_NAME}
  PUBLIC
  ${CMAKE_SOURCE_DIR}/src/cpp
)

# Add the static library
if(VCP_BUILD_STATIC)
  message("  [+] Building static ${LIB_NAME} library\n")
  set(LIBS ${LIBS} ${LIB_NAME}_static)
  add_library(${LIB_NAME}_static STATIC ${PUBLIC_HEADERS} ${PRIVATE_HEADERS} ${SOURCES})
  target_link_libraries(${LIB_NAME}_static 
    vcp_utils
    vcp_imutils
    vcp_math
    vcp_config
    ${ICC_EXTERNAL_LIBS}
  )
  target_include_directories(${LIB_NAME}
    PUBLIC
    ${CMAKE_SOURCE_DIR}/src/cpp
  )
else()
  message("  [ ] Won't build static ${LIB_NAME} library\n")
endif()


# Export (only public headers)
install(FILES ${PUBLIC_HEADERS}
  DESTINATION include/${PROJECT_NAME}
  COMPONENT headers
)
install(TARGETS ${LIBS}
  DESTINATION lib
  EXPORT ${LIB_NAME}
  COMPONENT libraries
)


