#!/bin/bash -

##############################################################################
# Look up VCP_ROOT_DIR and VCP_SCRIPT_DIR
##############################################################################
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
else
    VCP_SCRIPT_DIR="${VCP_ROOT_DIR}/scripts"
fi

echo
echo "[VCP] Found VCP_ROOT_DIR at '${VCP_ROOT_DIR}'"
echo


##############################################################################
# Ensure that all required packages are installed
##############################################################################
echo "[VCP] Checking requirements"
echo
"$VCP_SCRIPT_DIR"/setup-dependencies-ubuntu-18.04.sh
OPT_PKG_FLAGS=$?

# Check which of the optional components can be built
CMAKEOPTIONS=("-DCMAKE_BUILD_TYPE=Release" "-DVCP_BUILD_BEST=ON")
if [ $((OPT_PKG_FLAGS & 0x04)) -gt 0 ]; then
    echo "TODO enable RealSense!"
fi


if [ $((OPT_PKG_FLAGS & 0x08)) -gt 0 ]; then
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_K4A_STREAMING=ON")
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_K4A_STREAM_MJPG=OFF")
fi


if [ $((OPT_PKG_FLAGS & 0x10)) -gt 0 ]; then
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_HTTP_STREAMING=ON")
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_RTSP_STREAMING=ON")
    BUILD555=true
else
    BUILD555=false
fi


##############################################################################
# Set up external (non-packaged) libraries
##############################################################################
echo "[VCP] Preparing 3rd party libraries"
echo
# Remember current working directory to return here
CURR_WORK_DIR=$(pwd)
cd ${VCP_ROOT_DIR}/external
##############################################################################
##### live555 for RTSP stream handling
if [ "${BUILD555}" ]; then
  if [ ! -d "live" ]; then
      echo "  [VCP] Building live555"
      echo
      tar zxf live.2020.01.11.tar.gz
      cd live
      ./genMakefiles linux-64bit
      make -j
      ## I prefer not to '(make) install' live555. VCP takes care of adjusting
      ## the library linkage paths.
      cd ..
  fi
fi
##############################################################################
##### pybind11 for Python bindings
PYBIND_NAME=pybind11-2.4.3
if [ ! -d "${PYBIND_NAME}" ]; then
    echo "  [VCP] Building pybind11"
    echo
    tar zxf ${PYBIND_NAME}.tar.gz
    cd $PYBIND_NAME
    mkdir -p build
    cd build
    ## Set local(!) install path
    cmake -DPYBIND11_INSTALL=ON -DCMAKE_INSTALL_PREFIX=../install -DPYBIND11_TEST=OFF ..
    make install
    cd ../..
fi
##############################################################################
# CD back to VCP_ROOT_DIR
cd ..


##############################################################################
# Configure, build and (locally) install C++ libraries
##############################################################################
echo "  [VCP] Building VCP C++ libraries"
echo
mkdir -p build
cd build
cmake "${CMAKEOPTIONS[@]}" ..
make -j install
# CD back to VCP_ROOT_DIR
cd ..


##############################################################################
# Configure, build and (locally) install Python wrappers
##############################################################################
echo "  [VCP] Building VCP python3 bindings"
echo
cd src/python3
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j install
# CD back to VCP_ROOT_DIR
cd ../../..


##############################################################################
# Configure, build and (locally) install Python wrappers
##############################################################################
echo "  [VCP] Building C++ examples"
echo

cd examples
mkdir -p build
cd build
cmake ..
make -j
cd $CURR_WORK_DIR


