# Find nlohmann's "json for modern C++"

# We ship it within <vcp_root>/external/nlohmann
if(DEFINED ENV{VCP_ROOT_DIR})
    if(WIN32)
        string(REPLACE "\\" "/" VCP_ROOT $ENV{VCP_ROOT_DIR})
    else(WIN32)
        set(VCP_ROOT $ENV{VCP_ROOT_DIR})
    endif(WIN32)
else(DEFINED ENV{VCP_ROOT_DIR})
    set(VCP_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
    message(STATUS "[vcp-Findnlohmannjson] Environmental variable VCP_ROOT_DIR not set! Trying to continue from ${VCP_ROOT}")
endif(DEFINED ENV{VCP_ROOT_DIR})

set(POTENTIAL_INCLUDE_DIR ${VCP_ROOT}/external)

find_path(NLOHMANNJSON_INCLUDE_DIR nlohmann/json.hpp
    HINTS ${POTENTIAL_INCLUDE_DIR})

if (NLOHMANNJSON_INCLUDE_DIR)
    set(NLOHMANNJSON_FOUND TRUE)
endif (NLOHMANNJSON_INCLUDE_DIR)
