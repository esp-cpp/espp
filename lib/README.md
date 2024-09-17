# ESPP Library

This folder contains the configuration needed to cross-compile the central
(cross-platform) components of espp for the following platforms:

* PC (Linux, MacOS, Windows)
  * C++
  * Python (through pybind 11)

## Updating the python bindings

We use [litgen](https://github.com/pthom/litgen) to automatically parse specific
header files and generate python bindings for them in c++ using pybind11.

Relevant files:
- [./autogenerate_bindings.py](./autogenerate_bindings.py): This is the script
  which configures litgen and generates the appropriate code. Inside this script
  is where we tell litgen which header files to generate python bindings for.
  Note that the ordering of the header file list is important.
- [./python_bindings/pybind_espp.cpp](./python_bindings/pybind_espp.cpp): This
  is where the generated pybind11 c++ code will go. It will need to be modified
  after generation (see below).

### Setup

Create a virtual environment, and install the required packages:

``` console
python3 -m venv env
source env/bin/activate
pip install -r requirements.txt
```

### Generating python bindings

```console
# start the environment
source env/bin/activate
python autogenerate_bindings.py
```

Note: after you autogenerate the bindings, you will need to manually modify the
generated code in
[./python_bindings/pybind_espp.cpp](./python_bindings/pybind_espp.cpp) slightly
(at least until the underlying bugs in litgen/srcmlcpp are fixed):
1. You must fix the `RangeMapper::` to be `RangeMapper<int>::` and
   `RangeMapper<float>::` where appropriate.
2. You must fix the `pyClassRangeMapper,` to be `pyClassRangeMapper_int,` and
   `pyClassRangeMapper_float,` where appropriate (1 place each).
3. You must fix the `Bezier::` to be properly templated on `espp::Vector2f`
   (`Bezier<espp::Vector2f>::`).
4. You must fix the `pyClassBezier` to be instead `pyClassBezier_espp_Vector2f`.
5. You must fix the `Vector2d` generated template code for both `int` and
   `float` to ensure that the template type is always provided.
6. Srcml currently has an [issue with inner
   structs](https://github.com/srcML/srcML/issues/2033). This means that for
   `Bezier`, `Gaussian`, `Logger`, `Pid`, `Socket`, `Task`, `Timer`,
   `TcpSocket`, `UdpSocket`, and `Joystick`, litgen will improperly generate an
   `implicit default constructor`. You you can update the template parameter for
   it to have the appropriate `const espp::<ClassName>::Config&` template
   parameter. Note that for some classes, they may have multiple config
   options - so for `Bezier`, `Task`, `Timer`, etc. you will want to create
   overloads which target each of the config types.

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

