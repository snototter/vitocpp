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
echo "[vcp] Found VCP_ROOT_DIR at '${VCP_ROOT_DIR}'"
echo


##############################################################################
# Ensure that all required packages are installed
##############################################################################
echo "[vcp] Checking requirements"
echo
"$VCP_SCRIPT_DIR"/setup-dependencies-ubuntu-18.04.sh
OPT_PKG_FLAGS=$?

# Check which of the optional components can be built
CMAKEOPTIONS=("-DCMAKE_BUILD_TYPE=Release" "-DVCP_BUILD_BEST=ON")
if [ $((OPT_PKG_FLAGS & 0x04)) -gt 0 ]; then
    echo "TODO enable RealSense!"
else
    echo "TODO disable RealSense!"
fi


if [ $((OPT_PKG_FLAGS & 0x08)) -gt 0 ]; then
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_K4A=ON")
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_K4A_MJPG=OFF")
else
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_K4A=OFF")
fi


if [ $((OPT_PKG_FLAGS & 0x10)) -gt 0 ]; then
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_IPCAM=ON")
    BUILD555=true
else
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_IPCAM=OFF")
    BUILD555=false
fi


##############################################################################
# Set up external (non-packaged) libraries
##############################################################################
echo "[vcp] Preparing 3rd party libraries"
echo
# Remember current working directory to return here
CURR_WORK_DIR=$(pwd)
cd ${VCP_ROOT_DIR}/external
##############################################################################
##### live555 for RTSP stream handling
if [ "${BUILD555}" ]; then
  if [ ! -d "live" ]; then
      echo "  [vcp] Building live555"
      echo "  [vcp] Grabbing latest live555"
      echo "        If there are errors, use the frozen but known-to-be-working"
      echo "        ./external/live555<version>.tar.gz instead"
      echo
      wget http://www.live555.com/liveMedia/public/live555-latest.tar.gz
      tar zxf live555-latest.tar.gz
      cd live
      ./genMakefiles linux-64bit
      make -j > /dev/null
      ## I prefer not to '(make) install' live555. VCP takes care of adjusting
      ## the library linkage paths.
      cd ..
  fi
fi
##############################################################################
##### pybind11 for Python bindings
PYBIND_VERSION=2.4.3
PYBIND_NAME=pybind11-${PYBIND_VERSION}
if [ ! -d "${PYBIND_NAME}" ]; then
    echo "  [vcp] Building pybind11"
    echo "  [vcp] Grabbing pybind11 v${PYBIND_VERSION}"
    echo "        If there are errors, use the frozen but known-to-be-working"
    echo "        ./external/pybind11-<version>.tar.gz instead"
    echo
    wget https://github.com/pybind/pybind11/archive/v${PYBIND_VERSION}.tar.gz
    tar zxf v${PYBIND_VERSION}.tar.gz
    cd ${PYBIND_NAME}
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
echo "[vcp] Building VCP C++ libraries"
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
echo "[vcp] Building vcp python3 bindings"
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
echo "[vcp] Building C++ examples"
echo

cd examples
mkdir -p build
cd build
cmake ..
make -j
cd $CURR_WORK_DIR


