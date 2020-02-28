# vitocpp/vcp
C++/Python 3 utilities for common vision tasks, _e.g._ streaming, visualization or image manipulation.

<b>Note:</b> As of 01/2020 this repository is <b>WIP</b>, I'm rewriting my utilities (especially simplifying the streaming module) over the next couple of months.

```
Current status:
cloc --exclude-dir=.venv3,build,third-party,gen,doc --exclude-lang=make,XML .
-------------------------------------------------------------------------------
Language                     files          blank        comment           code
-------------------------------------------------------------------------------
C++                             59           3626           2414          18863
C/C++ Header                    49           1411           1561           3799
CMake                           21            374            446           1691
Python                          18            484            689           1455
Markdown                         3             52              0            256
Bourne Shell                     3             37             70            223
-------------------------------------------------------------------------------
SUM:                           153           5984           5180          26287
-------------------------------------------------------------------------------
```

## What is it good for?
Some of `vcp`'s highlights:
* <b>Best effort multiple device streaming</b> with the `vcp::best` module. Useful for quick camera tests and whenever (guaranteed) synchronisation isn't crucial. The goal of this module is to stream from multiple devices simultaneously, without having the ROS overhead. A simple libconfig++-style configuration file allows you to process all streams both in C++ and python. For more details, see the [separate BESt.md documentation](BESt.md).
* <b>Background subtraction</b> is provided by the `vcp::bgm` module, for example:

  ![Background Subtraction](https://github.com/snototter/vitocpp/blob/master/doc/example-bgm.png)
* <b>Visualization utilities</b> are provided by the `vcp::imvis` module, for example:

  ![Visualization Example](https://github.com/snototter/vitocpp/blob/master/doc/example-imvis.png)
* <b>Pseudocoloring</b> for data visualization/analysis is also provided by the `vcp::imvis` module.
  Back when I started working on this library, OpenCV didn't provide pseudocoloring capabilities. The `vcp::imvis` module allows visualization via common color maps (you might remember these from such frameworks as MATLAB, matplotlib, and whatnot).

  ![Pseudocoloring Example](https://github.com/snototter/vitocpp/blob/master/doc/example-pseudocolor.png)
* <b>Math</b> utilities - besides enabling most of the fancy visualizations within `vcp::imvis`, you can also do basic geometry tasks with the `vcp::math` module (_e.g._ computing tangents of circles, line (segment) intersection, and quite a lot more).

  ![Geometry Example](https://github.com/snototter/vitocpp/blob/master/doc/example-geometry.png)
* <b>C++</b> utilities - for basic file/path and string manipulation, sorting, and more (if you want to avoid heavier dependencies, such as Boost).


## Repository Contents
* `./build-scripts` - shell scripts to prepare the build system, build the libraries, etc.
* `./cmake` - CMake utilities (incl. custom `Find<package>.cmake` scripts).
* `./examples` - example usage of these utilities.
* `./src` - source code for all C++ modules and python bindings.
* `./third-party` - third party dependencies (which are not part of default system packages).


## Installation
* If you want to build the C++ library and Python3 bindings, simply run `./build-scripts/build-ubuntu-18.04.sh`, this will:
  * Check and ask you to install missing system packages.
  * Configure the build (depending on which optional packages you installed, _e.g._ `libk4a` to stream from Azure Kinect).
  * Build the C++ `vcp` libraries and corresponding Python3 bindings.
  * Build the example applications and prepare the virtual environment for Python demos.
* If you prefer to do it on your own, crank up CMake:
    ```bash
    $ cd <VCP_ROOT_DIR>
    $ mkdir build && cd build
    $ cmake -DVCP_BUILD_PYTHON=ON ..
    $ make -j install
    ```
* The default install location (for `make install`) of `vcp` is `<VCP_ROOT_DIR>/gen`.
  * Public C++ headers are inside `<VCP_ROOT_DIR>/gen/include`.
  * `vcp` library files are inside `<VCP_ROOT_DIR>/gen/lib`.
  * The Python3 package is located at `<VCP_ROOT_DIR>/gen/vcp`.


## Tools
This repository comes with a few "tools", _i.e._ standalone applications that go beyond simple demos/examples.
Currently (as of Jan/2020), there are only python applications available, which you can find at `<VCP_ROOT_DIR>/examples/python3/tools`.
* Set up the <b>virtual environment</b>. If you didn't use the `./build-scripts/build-X.sh` script to prepare the library, you should run:
  ```bash
  $ cd <VCP_ROOT_DIR>/examples/python3
  $ ./prepare_environment_py3.sh
  $ source .venv3/bin/activate
  $ cd tools
  ```
* <b>3D image plots</b>. Plot a single image (`plot-image.py`) or an image sequence (`plot-image-sequence.py`) by interactively adjusting the camera extrinsics.

  ![Image Sequence Example](https://github.com/snototter/vitocpp/blob/master/doc/example-render-img-sequence.png)
* <b>Image abstraction:</b> cartoonification, pixelation, etc. can be experimented with via `cartoonify.py`.


## Examples
Besides the more complex `tools`, there are also simple demos showcasing the library capabilities.
### C++
* The C++ applications at `<VCP_ROOT_DIR>/examples/cpp` demonstrate how you can use `vcp` from your own CMake projects. After building the vcp library, building these examples is as easy as:
  ```bash
  $ cd <VCP_ROOT_DIR>/examples
  $ mkdir build && cd build
  $ cmake ..
  $ make -j
  ```
* Highly recommended examples (best/well documented, useful functionality):
  * `best_demo`


### Python
* You can find Python3 examples for all modules at `<VCP_ROOT_DIR>/examples/python3`.
  ```bash
  $ cd <VCP_ROOT_DIR>/examples/python3
  # Set up the virtual environment
  $ ./prepare_environment_py3.sh
  $ source .venv3/bin/activate
  $ python imvis_demo.py
  ```
* Highly recommended examples (best/well documented, useful functionality):
  * `bgm_demo.py` - Usage example of the background subtraction module.
  * `geometry_demoy.py` - Usage example of the `math2d` and `math3d` modules.
  * `imutils_demo.py` - Usage example of the `imutils` (image manipulation) module.
  * `imvis_demo.py` - Usage example for visualization utilities within the `imvis` module.

## Tests
While all of vcp has been tested "in-the-wild", unit tests are rather sparse, unfortunately.
Especially for the "best effort streaming" module, tests become quite difficult to automate (threading + the need for the specific hardware connected to the test server).

### C++
* Testing the C++ core library requires `gtest`:
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
  $ cd <VCP_ROOT_DIR>/build

  # Enable tests and link the gtest libraries
  $ cmake -DVCP_BUILD_TESTS=ON ..
  $ make test 

  # Or, if you prefer:
  $ ctest -V
  ```

### Python
* You can find the Python3 tests at `<VCP_ROOT_DIR>/examples/python3`, _i.e._ all files following the pattern `test_*.py`.
* Testing requires `pytest`:
  ```bash
  $ cd <VCP_ROOT_DIR>/examples/python3
  $ source .venv3/bin/activate
  $ pip install pytest
  $ pytest
  ```

## TODOs
* [ ] BESt module
  * [ ] Load intrinsics
    * [ ] Mono
    * [ ] Stereo
    * [ ] RGB+D K4A
    * [x] RGB+D RealSense
  * [ ] Load extrinsics
  * [ ] Rectify streams
    * [ ] Mono
    * [ ] Stereo
    * [ ] RGB+D K4A
    * [x] RGB+D RealSense
  * [ ] Azure Kinect
    * [x] Stream raw data
    * [x] Align RGB+D
    * [ ] Query intrinsics
  * [ ] mvBlueFox
  * [ ] Mobotix, etc.
* [ ] Increase unit test coverage
* [ ] Tools/examples
  * [ ] C++ Viewer (resize streams and use liveview)
  * [ ] Qt Viewer
  * [ ] Capturing tool
* [ ] Camera calibration (nice-to-have)
* [ ] Tracking module

