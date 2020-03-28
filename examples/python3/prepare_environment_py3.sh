#!/bin/bash --

# Virtual environment
venv=.venv3
if [ ! -d "${venv}" ]
then
  echo "Setting up virtual environment"
  python3 -m venv ${venv}
  source ${venv}/bin/activate
  pip3 install --upgrade pip
  pip3 install -r ../../src/python3/requirements.txt
  pip3 install matplotlib
  pip3 install pytest
  pip3 install apriltag

  # Set up OpenCV - assumes that you already installed it!
  echo "Trying to link to your OpenCV installation"
  opencv_lib=$(find /usr -name cv2* | grep python3 | head -n 1)
  if [ -z "${opencv_lib}" ]
  then
    echo "[E] You need to install OpenCV first!" 1>&2
    exit 23
  fi
  # Get correct python subfolder
  pverstring=$(ls ${venv}/lib/ | grep python3)
  libdir=$(dirname "${opencv_lib}")
  #echo $libdir
  #echo $pverstring
  # Create link file in virtualenv
  echo ${libdir} > ${venv}/lib/${pverstring}/site-packages/cv2.pth  
fi


# Get the directory this file is located in (only needed for the following stdout message).
# Taken from https://stackoverflow.com/a/246128/400948
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
    DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
    SOURCE="$(readlink "$SOURCE")"
    # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
    [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
currdir="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
echo
echo "################################################################"
echo
echo "  Don't forget to activate your virtual environment:"
echo
echo "    source ${currdir}/${venv}/bin/activate"
echo
echo "################################################################"
echo

