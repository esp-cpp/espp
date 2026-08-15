#!/bin/bash

mkdir build
cd build
# ESPP_BUILD_PYTHON=ON builds and installs the python package into lib/pc
# alongside the C++ static library + headers (this is what CI publishes).
cmake -DESPP_BUILD_PYTHON=ON ..
cmake --build . --config Release --target install --parallel 4
