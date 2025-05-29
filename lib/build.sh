#!/bin/bash

mkdir build
cd build
cmake ..
cmake --build . --config Release --target install --parallel 4
