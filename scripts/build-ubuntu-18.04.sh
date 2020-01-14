#!/bin/bash -

##############################################################################
# Ensure that all required packages are installed
##############################################################################
./setup-dependencies-ubuntu-18.04.sh
OPT_PKG_FLAGS=$?

# Check which of the optional components can be built
CMAKEOPTIONS=("-DCMAKE_BUILD_TYPE=Release")
if [ $((OPT_PKG_FLAGS & 0x04)) -gt 0 ]; then
    echo "TODO enable RealSense!"
fi

if [ $((OPT_PKG_FLAGS & 0x08)) -gt 0 ]; then
    CMAKEOPTIONS+=("-DWITH_K4A")
fi


##############################################################################
# Prepare third party content
##############################################################################
# We need to know the VCP_ROOT_DIR first.
if [ -z "$VCP_ROOT_DIR" ]; then
  # Get the directory this file is located in.
  # Taken from https://stackoverflow.com/a/246128/400948
  SOURCE="${BASH_SOURCE[0]}"
  while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
    DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
    SOURCE="$(readlink "$SOURCE")"
    # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
    [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
  done
  VCP_SCRIPT_DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
  VCP_ROOT_DIR="${VCP_SCRIPT_DIR}/.."
fi
# Remember current working directory to return here
CURR_WORK_DIR=$(pwd)

cd ${VCP_ROOT_DIR}/external
##### live555 for RTSP stream handling
if [ ! -d "live" ]; then
    tar zxf live.2020.01.11.tar.gz
    cd live
    ./genMakefiles linux-64bit
    make -j
    ## I prefer not to '(make) install' live555. VCP takes care of adjusting
    ## the library linkage paths.
    cd ..
fi

##### pybind11 for Python bindings
PYBIND_NAME=pybind11-2.4.3
if [ ! -d "${PYBIND_NAME}" ]; then
    tar zxf ${PYBIND_NAME}.tar.gz
    cd $PYBIND_NAME
    mkdir -p build
    cd build
    ## Set local(!) install path
    cmake -DPYBIND11_INSTALL=ON -DCMAKE_INSTALL_PREFIX=../install -DPYBIND11_TEST=OFF ..
    make install
    cd ../..
fi

# CD back to VCP_ROOT_DIR
cd ..


##############################################################################
# Configure, build and (locally) install C++ libraries
##############################################################################
mkdir -p build
cd build

cd $CURR_WORK_DIR
#cmake "${CMAKEOPTIONS[@]}" ..



##############################################################################
# Configure, build and (locally) install Python wrappers
##############################################################################

