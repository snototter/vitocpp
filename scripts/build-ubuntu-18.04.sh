#!/bin/bash -

##############################################################################
# Ensure that all required packages are installed
##############################################################################
./setup-dependencies-ubuntu-18.04.sh
optional_package_flags=$?

# Check which of the optional components can be built
CMAKEOPTIONS=("-DCMAKE_BUILD_TYPE=Release")
if [ $((optional_package_flags & 0x04)) -gt 0 ]; then
    echo "TODO enable RealSense!"
fi

if [ $((optional_package_flags & 0x08)) -gt 0 ]; then
    CMAKEOPTIONS+=("-DWITH_K4A")
fi


##############################################################################
# Prepare third party content
##############################################################################


##############################################################################
# Configure, build and (locally) install C++ libraries
##############################################################################
#mkdir -p build
#cd build
#cmake "${CMAKEOPTIONS[@]}" ..



##############################################################################
# Configure, build and (locally) install Python wrappers
##############################################################################

