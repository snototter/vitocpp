#!/bin/bash -

# Exit upon error
set -e

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
    VCP_SCRIPT_DIR="${VCP_ROOT_DIR}/build-scripts"
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
CMAKEOPTIONS=("-DCMAKE_BUILD_TYPE=Release" "-DVCP_BUILD_BEST=ON" "-DVCP_BUILD_PYTHON=ON")

# RealSense2
if [ $((OPT_PKG_FLAGS & 0x04)) -gt 0 ]; then
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_REALSENSE2=ON")
else
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_REALSENSE2=OFF")
fi

# Kinect for Azure
if [ $((OPT_PKG_FLAGS & 0x08)) -gt 0 ]; then
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_K4A=ON")
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_K4A_MJPG=OFF")
else
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_K4A=OFF")
fi

# IP cameras (HTTP & RTSP)
if [ $((OPT_PKG_FLAGS & 0x10)) -gt 0 ]; then
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_IPCAM=ON")
    BUILD555=true
else
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_IPCAM=OFF")
    BUILD555=false
fi

# Zed camera
if [ $((OPT_PKG_FLAGS & 0x20)) -gt 0 ]; then
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_ZED=ON")
else
    CMAKEOPTIONS+=("-DVCP_BEST_WITH_ZED=OFF")
fi


##############################################################################
# Set up third-party dependencies
##############################################################################
echo "[vcp] Preparing 3rd party libraries"
echo
# Remember current working directory to return here
CURR_WORK_DIR=$(pwd)
cd ${VCP_ROOT_DIR}/third-party
# Limit number of cores (I experienced issues when building the python
# bindings on an Intel i5 Quad Core processor without limitation).
np=$(nproc)
ncores=$((np-2))
ncores=$(( ncores > 0 ? ncores : 1))
##############################################################################
##### live555 for RTSP stream handling
if [ "${BUILD555}" ]; then
  if [ ! -d "live" ]; then
      echo "  [vcp] Grabbing latest live555"
      echo "        If there are errors, use the frozen but known-to-be-working"
      echo "        ./third-party/live555<version>.tar.gz instead"
      echo
      wget http://www.live555.com/liveMedia/public/live555-latest.tar.gz
      tar zxf live555-latest.tar.gz
      cd live      
      echo "  [vcp] Building live555"
      ./genMakefiles linux-64bit
      make -j$ncores >> live555-build.log 2>&1
      if [ $? -ne 0 ]; then
          echo "  [vcp] Cannot build live555!"
          echo "        Check ${VCP_ROOT_DIR}/third-party/live/live555-build.log"
          cd $CURR_WORK_DIR
          exit 23
      fi
      rm live555-latest.tar.gz
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
    echo "  [vcp] Grabbing pybind11 v${PYBIND_VERSION}"
    echo "        If there are errors, use the frozen but known-to-be-working"
    echo "        ./third-party/pybind11-<version>.tar.gz instead"
    echo
    wget https://github.com/pybind/pybind11/archive/v${PYBIND_VERSION}.tar.gz
    archive=v${PYBIND_VERSION}.tar.gz
    tar zxf ${archive}
    echo "  [vcp] Setting up pybind11"
    cd ${PYBIND_NAME}
    mkdir -p build
    cd build
    ## Set local(!) install path
    cmake -DPYBIND11_INSTALL=ON -DCMAKE_INSTALL_PREFIX=../install -DPYBIND11_TEST=OFF ..
    make install
    if [ $? -ne 0 ]; then
          echo "  [vcp] Cannot set up pybind11!"
          cd $CURR_WORK_DIR
          exit 24
      fi
    cd ../..
    rm ${archive}
fi
##############################################################################
# CD back to VCP_ROOT_DIR
cd ..


##############################################################################
# Configure, build and (locally) install C++ libraries and Python bindings
##############################################################################
echo "[vcp] Building VCP C++ libraries & Python bindings"
echo
mkdir -p build
cd build
cmake "${CMAKEOPTIONS[@]}" .. && make -j$ncores install
rc_vcp_lib=$?
# CD back to VCP_ROOT_DIR
cd ..


if [ $rc_vcp_lib -ne 0 ]; then
    echo "[vcp] Library build failed!"
    echo "      Check the build output."
else
    ##############################################################################
    # Prepare the pth file
    ##############################################################################
    echo "[vcp] Preparing the .pth file for inclusion in your virtualenv's site-packages"
    VCP_PTH=${VCP_ROOT_DIR}/gen/vcp.pth
    echo "${VCP_ROOT_DIR}/gen" > $VCP_PTH
    echo "      Please use ${VCP_PTH}"
    echo 
    

    ##############################################################################
    # Prepare the examples/tools
    ##############################################################################
    echo "[vcp] Building C++ examples"
    echo

    cd examples
    mkdir -p build
    cd build
    cmake .. && make -j$ncores
    cd ..

    echo "[vcp] Setting up Python3 virtual environment"
    echo
    cd python3
    /bin/bash prepare_environment_py3.sh
fi

# Go back to where we started
cd $CURR_WORK_DIR

