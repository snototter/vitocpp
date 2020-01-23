# vitocpp (VCP)
C++/Python 3 utilities for common vision tasks, e.g. streaming, visualization or image manipulation.

<b>Note:</b> As of 01/2020 this repository is <b>WIP</b>, I'm rewriting my utilities (especially simplifying the streaming module) over the next couple of months.

## What is it good for?
Some of `vcp`'s highlights:
* Best effort multiple device streaming. Useful for quick camera tests and whenever (guaranteed) synchronisation isn't crucial. The goal of this module is to stream from multiple devices simultaneously, without having the ROS overhead. A simple configuration file like this:
  ```C++
  TODO add
  ```
  allows you to process all streams both in C++ and python. For more details, look into the provided examples.
  TODO add separate BESt README to show more complex configs (e.g. adjusting RealSense/K4A/MatrixVision camera parameters).
* Nice(r) visualizations with less effort than plain OpenCV (have you ever tried to render a 3D bounding box?)
TODO example images (drawingXY)
* Pseudocoloring for data visualization/analysis.
TODO (+ see iminspect)
* Geometry utilities - besides enabling all the visualizations above, you can also do basic geometry tasks (e.g. computing tangents of circles, line (segment) intersection, and quite a lot more).
* C++ utilities - for basic file/path and string manipulation (if you want to avoid heavier dependencies, such as Boost).


## Repository Contents
* `./cmake` - CMake utilities (incl. custom `Find<package>.cmake` scripts).
* `./examples` - example usage of these utilities.
* `./external` - third party libraries (which cannot be installed out-of-the-box).
* `./scripts` - shell scripts to prepare the build system, build the libraries, and use/include the library in your project.
* `./src` - source for all modules and python bindings.


## Installation
* If you want to build the C++ library and Python 3 bindings, simply run `./scripts/build-ubuntu-18.04.sh`, this will:
  * Check and ask you to install missing system packages.
  * Configure the build (depending on which optional packages you installed, e.g. `libk4a` to stream from Azure Kinect).
  * Build the vcp libraries.
  * Build the python bindings.
  * Build the example applications.
* If you've already installed the required packages, and simply want to build the Clibrary, it's as easy as
  ```bash
  # Build the C++ library
  $ cd $VCP_ROOT_DIR
  $ mkdir build && cd build
  $ cmake ..
  $ make -j install
  # Afterwards, all library files can be found at <VCP_ROOT_DIR/gen>

  # Build the Python bindings
  $ cd $VCP_ROOT_DIR/src/python3
  $ mkdir build && cd build
  $ cmake ..
  $ make -j install
  # Afterwards, the Python package can be loaded from <VCP_ROOT_DIR/src/python3/vcp>
  ```
  #TODO install python package in /gen/python3?


## Examples
### C++
* The C++ applications at `<VCP_ROOT_DIR>/examples/cpp` demonstrate how you can use `vcp` from your own CMake projects. After building the vcp library, building these examples is as easy as:
  ```bash
  $ cd $VCP_ROOT_DIR/examples
  $ mkdir build && cd build
  $ cmake ..
  $ make -j
  ```
* Highly recommended examples (best/well documented, useful functionality):
  * `best_demo`
  * `imvis_demo` TODO
  * `imutils_demo` TODO


### Python
* You can find Python3 examples for all modules at `$VCP_ROOT_DIR/examples/python3`.
  ```bash
  $ cd $VCP_ROOT_DIR/examples/python3
  # Set up the virtual environment
  $ ./prepare_environment_py3.sh
  $ source .venv3/bin/activate
  $ python imvis_demo.py
  ```
* Highly recommended examples (best/well documented, useful functionality):
  * To be done

## Tests
While all of vcp has been tested "in-the-wild", unit tests are rather sparse, unfortunately.
Especially for the "best effort streaming" module, tests become rather impossible to automate (threading + the need for the specific hardware connected to the test server).

Testing requires `gtest`, which you'll probably need to build yourself - this is a no-brainer, see:
* Set up `libgtest`:
  ```bash
  $ sudo apt-get install libgtest-dev

  # This package does not provide binaries, so look for the installed
  # source at /usr/src. It should be (Ubuntu 18.04) at /usr/src/googletest.
  $ cd /usr/src/googletest
  $ sudo cmake CMakeLists.txt
  $ sudo make

  # Copy or symlink googlemock/gtest/libgtest.a 
  # and googlemock/gtest/libgtest_main.a to your /usr/lib folder
  $ sudo cp googlemock/gtest/*.a /usr/lib
  ```
* Now `vcp` can be tested:
  ```bash
  $ cd $VCP_ROOT_DIR/build

  # Enable tests and link the gtest libraries
  $ cmake -DVCP_BUILD_TESTS ..
  $ make test 

  # Or, if you prefer:
  $ ctest -V
  ```


##TODOs
* [ ] BGM module
  * Factory pattern!
* [ ] BESt module
  * [ ] Load intrinsics
  * [ ] Load extrinsics
  * [ ] Rectify streams
  * [x] File sinks
  * [x] Webcam sink
  * [x] HTTP sink
  * [ ] RTSP sink
  * [ ] Monocular/stereo
    * [x] Stereo IP cam streams as separate monocular sinks
    * [ ] "Real" stereo cams (e.g. ZED) could be split in VCP or by the library user (currently, the latter is preferred)
  * [ ] RealSense
  * [ ] Azure Kinect
    * [x] Stream raw data
    * [x] Align RGB+D
    * [ ] Query intrinsics
  * [ ] mvBlueFox
  * [ ] Axis, Mobotix, etc.
* [ ] Python bindings
  * [ ] best
  * [ ] config - likely not needed
  * [x] imutils
  * [x] imvis
  * [x] math
  * [ ] ui
  * [x] utils
* [ ] Move python demos (e.g. math2d/3d) to unit tests
* [ ] Increase unit test coverage
* [ ] C++ Tools/examples
  * [ ] Viewer
  * [ ] Capturing tool
* [ ] Camera calibration (nice-to-have)
* [ ] Tracking module

