# Find nlohmann's "json for modern C++"

# We ship it within <pvt_root>/third_party/nlohmann
if(DEFINED ENV{PVT_ROOT_DIR})
  if(WIN32)
    string(REPLACE "\\" "/" PVT_ROOT $ENV{PVT_ROOT_DIR})
  else(WIN32)
    set(PVT_ROOT $ENV{PVT_ROOT_DIR})
  endif(WIN32)
else(DEFINED ENV{PVT_ROOT_DIR})
  set(PVT_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
  message(WARNING "Environmental variable PVT_ROOT_DIR not set! Trying to continue from ${PVT_ROOT}")
endif(DEFINED ENV{PVT_ROOT_DIR})

set(POTENTIAL_INCLUDE_DIR ${PVT_ROOT}/third_party)

find_path(NLOHMANNJSON_INCLUDE_DIR nlohmann/json.hpp
  HINTS ${POTENTIAL_INCLUDE_DIR})

if (NLOHMANNJSON_INCLUDE_DIR)
  set(NLOHMANNJSON_FOUND TRUE)
endif (NLOHMANNJSON_INCLUDE_DIR)
