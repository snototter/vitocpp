# vitocpp (VCP)
C++/Python 3 utilities for common vision tasks, e.g. streaming, visualization or image manipulation.


## What is it good for?
Some of `vcp`'s highlights:
* Best effort multi-device streaming. Suppose you have a capturing setup similar to this:
  ```
    Stream  Label         Type        Config. Param
  --------------------------------------------------
         0  kinect-rgb    rgbd-image  sink-k4a  
         1  kinect-depth  rgbd-depth  sink-k4a
         2  zed-stereo    stereo      sink-webcam 
         3  ip-cam-axis   monocular   sink-axis
  --------------------------------------------------
    4 streams from 4 devices
    * Devices are available
    * Frames are enqueued
  ```
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
* If you've already installed the required packages, and simply want to build the library, it's as easy as
  ```bash
  $ cd $VCP_ROOT_DIR
  $ mkdir build && cd build
  $ cmake ..
  $ make -j install
  # Afterwards, all library files can be found at <$VCP_ROOT_DIR/gen>
  ```


## Examples
### C++
* The C++ applications at `$VCP_ROOT_DIR/examples/cpp` demonstrate how you can use `vcp` from your own CMake projects. After building the vcp library, building these examples is as easy as:
  ```bash
  $ cd $VCP_ROOT_DIR/examples
  mkdir build && cd build
  cmake ..
  make -j
  ```


### Python
To be done!


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
  * [ ] HTTP sink
  * [ ] RTSP sink
  * [ ] Monocular/stereo
  * [ ] RealSense
  * [ ] Azure Kinect
    * [ ] Stream raw data
    * [ ] Align RGB+D
    * [ ] Query intrinsics
  * [ ] mvBlueFox
  * [ ] Axis, Mobotix, etc.
* [ ] Python bindings
* [ ] Increase unit test coverage
* [ ] C++ Tools/examples
  * [ ] Viewer
  * [ ] Capturing tool
* [ ] Camera calibration (nice-to-have)
* [ ] Tracking module

