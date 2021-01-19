#!/bin/bash -

# Function definition
function is_package_installed {
# Returns 1 if package is installed, 0 otherwise
    pkg=$1
    if dpkg --get-selections | grep -q "^$pkg\(\|:[^\s]*\)*[[:space:]]*install$" >/dev/null; then
        return 1
    else
        return 0
    fi
}


function check_optional_packages {
# Returns 1 if all given packages are installed, 0 otherwise
    for pkg in "$@"; do
        is_package_installed $pkg
        if [ $? -eq 0 ]; then
            printf "  Optional package '%s' is not installed.\n" "$pkg"
            return 0
        fi
    done
    return 1
}


##############################################################################
# Check default packages required to build the basic VCP library
##############################################################################
PKG_DEFAULT="cmake build-essential libopencv-dev python3-opencv zlib1g-dev libconfig++-dev ffmpeg python3-dev python3-pip python3-venv libopenblas-base libopenblas-dev libeigen3-dev libatlas-base-dev libatlas3-base"

# Check, which packages are missing...
PKG_INSTALL=()
printf "  Checking dependencies for VCP:\n"
for pkg in ${PKG_DEFAULT}; do
    is_package_installed $pkg
    if [ $? -eq 0 ]; then
        printf "    Need to install package '%s'\n" "$pkg"
        PKG_INSTALL+=($pkg)
    fi
done

# ... now install those
if [ "${#PKG_INSTALL[@]}" -gt 0 ]; then
    printf "  Installing missing default packages:\n"
    for pkg in ${PKG_INSTALL[@]}; do
        if sudo apt -qq install $pkg; then
            printf "    Successfully installed $pkg\n"
        else
            printf "    Error installing $pkg\n"
            exit 1
        fi
    done
else
    printf "    Default package requirements satisfied.\n"
fi


##############################################################################
# Check optional packages
# e.g. required to support streaming from different devices, such as
# RealSense, Azure Kinect, MatrixVision, etc.
##############################################################################
# We use the script's exit code to configure the build (i.e. to indicate
# which optional packages should be installed)
return_code=0

# Intel RealSense
# Dependencies are satisfied if 3rd bit of return code is set
PKG_REALSENSE=(librealsense2 librealsense2-dev librealsense2-dkms)
echo
printf "  Checking optional packages for Intel RealSense support:\n"
check_optional_packages "${PKG_REALSENSE[@]}"
if [ $? -eq 1 ]; then
    printf "    All satisfied.\n  => Did you check /etc/default/grub for large enough usbcore.usbfs_memory_mb setting?\n"
    return_code=$((return_code | 0x04))
fi

# Azure Kinect
# Dependencies are satisfied if 4th bit of return code is set
PKG_KINECT=(libk4a1.3 libk4a1.3-dev)
echo
printf "  Checking optional packages for Azure Kinect support (libk4a1.3):\n"
check_optional_packages "${PKG_KINECT[@]}"
if [ $? -eq 1 ]; then
    printf "    All satisfied.\n  => Did you check /etc/default/grub for large enough usbcore.usbfs_memory_mb setting?\n"
    return_code=$((return_code | 0x08))
fi
# Also check newer versions
PKG_KINECT=(libk4a1.4 libk4a1.4-dev)
echo
printf "  Checking optional packages for Azure Kinect support (libk4a1.4):\n"
check_optional_packages "${PKG_KINECT[@]}"
if [ $? -eq 1 ]; then
    printf "    All satisfied.\n  => Did you check /etc/default/grub for large enough usbcore.usbfs_memory_mb setting?\n"
    return_code=$((return_code | 0x08))
fi

# IP camera streaming
# Dependencies are satisified if 5th bit of the return code is set
PKG_IPC=(libcurl4-openssl-dev libssl-dev libavcodec-dev libavformat-dev libavutil-dev libavdevice-dev libswscale-dev)
echo
printf "  Checking optional packages for IP camera streaming (HTTP and RTSP):\n"
check_optional_packages "${PKG_IPC[@]}"
if [ $? -eq 1 ]; then
    printf "    All satisfied.\n"
    return_code=$((return_code | 0x10))
fi

# ZED camera streaming
# Requires a separate SDK, which is installed by default at /usr/local/zed
echo
zedpath=/usr/local/zed/
printf "  Checking StereoLabs Zed SDK at '${zedpath}':\n"
if [ -d "${zedpath}" ]; then
    printf "    SDK found, enabling Zed streaming.\n"
    return_code=$((return_code | 0x20))
else
    printf "    SDK NOT found, disabling Zed streaming.\n"
fi

exit $return_code

