# vitocpp (VCP)
C++/Python 3 utilities for common vision tasks, e.g. streaming, visualization or image manipulation.

## Repository Contents
* `cmake` - CMake utilities (incl. custom `Find<package>.cmake` scripts).
* `examples` - example usage of these utilities.
* `external` - third party libraries (which cannot be installed out-of-the-box).
* `scripts` - shell scripts to prepare the build system, build the libraries, and use/include the library in your project.
* `src` - source for all modules and python bindings.


## Subfolder "scripts"
* If you want to build the C++ library and Python 3 bindings, simply run `./scripts/build-ubuntu-18.04.sh`


#TODOs
* [ ] Rename (e.g. "visualization" => imvis)
* [ ] libconfig++ wrapper
* [ ] Build script
* [ ] BGMs
* [ ] BEST
* [ ] Math
* [ ] imutils
* [ ] imvis
* [ ] "ui"
* [ ] Python bindings


#TODO doc testing
set up testing
Testing the C++ library requires gtest, be sure to build the static libraries:
* Set up `libgtest` (must be compiled by yourself):
  ```bash
  $ sudo apt-get install libgtest-dev
  $ cd /usr/src/googletest
  $ sudo cmake CMakeLists.txt
  $ sudo make
  $ # copy or symlink googlemock/gtest/libgtest.a and googlemock/gtest/libgtest_main.a to your /usr/lib folder
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

TODO examples/c++
examples/cpp demonstrate how to use vitocpp/VCP from your own cmake project
```bash
$ cd $VCP_ROOT_DIR/examples
mkdir build && cd build
```

