# ESPP Library

This folder contains the configuration needed to cross-compile the central
(cross-platform) components of espp for the following platforms:

* PC (Linux, MacOS, Windows)
  * C++
  * Python (through pybind 11)

## Building for PC (C++ & Python)

To build the library for use on PC (with C++ and Python), simply build with
cmake:

``` sh
mkdir build
cd build
cmake ..
cmake --build . --config Release --target install
```

This is conveniently scripted up for you into [./build.sh](./build.sh) and
[./build.ps1](./build.ps1) scripts you can simply run from your terminal.

This will build and install the following files:

* `./pc/libespp_pc` - C++ static library for use with other C++ code.
* `./pc/include` - All the header files need for using the library from C++ code.
* `./pc/espp.so` - C++ shared library for python binding for use with python code.

