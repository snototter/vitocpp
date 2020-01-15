# vitocpp (VCP)
C++/Python 3 utilities for common vision tasks, e.g. streaming, visualization or image manipulation.

## Repository Contents
* `cmake` - CMake utilities (incl. custom `Find<package>.cmake` scripts).
* `examples` - example usage of these utilities.
* `external` - third party libraries (which cannot be installed out-of-the-box).
* `scripts` - shell scripts to prepare the build system, build the libraries, and use/include the library in your project.
* `src` - source for all modules and python bindings.

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
While all of vcp has been tested "in-the-wild", unit tests are quite sparse, unfortunately.
It's on my TODO list to improve code coverage. However, this project is not my first priority.
Feel free to add unit tests yourself and send a PR.

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
* [ ] BGMs
* [ ] BEST
* [ ] Python bindings
* [ ] Increase unit test coverage

