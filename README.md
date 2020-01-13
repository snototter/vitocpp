# vitocpp (VCP)
C++/Python 3 utilities for common vision tasks, e.g. streaming, visualization or image manipulation.

## Repository Contents
* `cmake` - CMake utilities (incl. custom `Find<package>.cmake` scripts)
* `external` - all used third party libraries (which cannot be installed out-of-the-box)
* `scripts` - shell scripts to prepare the build system, build the libraries, and use/include the library in your project.


## Subfolder "scripts"
* If you want to build the C++ library and Python 3 bindings
  ```bash
  $ cd $VCP_ROOT_DIR/scripts
  $ ./build-ubuntu-18.04.sh
  ```
#TODOs
* [ ] Set up CMake build system, adjusted for VCP
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
