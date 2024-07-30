#!/bin/bash

mkdir build
cd build
cmake ..
cmake --build .
cmake --install .
