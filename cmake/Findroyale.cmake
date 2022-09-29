# Sets the CMAKE variables for the experimental PMD support.
#  
#  royale_FOUND
#  royale_INCLUDE_DIRS
#  royale_LIB_DIR
#  royale_LIBS
#
# Don't use royale's config.cmake - incompatible compiler settings!

message(WARNING "Using absolute paths for experimental PMD support!")

set(royale_INSTALL_PATH "/opt/pmd") # TODO make user adjustable

set (royale_INCLUDE_DIRS "${royale_INSTALL_PATH}/include")
set (royale_LIB_DIR "${royale_INSTALL_PATH}/bin")
set (royale_LIBS "royale")

# Check if all include dirs exist
set(royale_FOUND true)
foreach(INC_DIR ${royale_INCLUDE_DIRS})
  if(NOT EXISTS ${INC_DIR})
    set(royale_FOUND false)
    message(FATAL_ERROR "Cannot find royale include dir: " ${INC_DIR})
  endif()
endforeach()

