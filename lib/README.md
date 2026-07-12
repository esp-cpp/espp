# ESPP Library

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [ESPP Library](#espp-library)
  - [Description](#description)
  - [Installing the python package with pip](#installing-the-python-package-with-pip)
  - [Building for PC (C++ & Python)](#building-for-pc-c--python)
  - [Updating the python bindings](#updating-the-python-bindings)
    - [Setup](#setup)
    - [Generating python bindings](#generating-python-bindings)

<!-- markdown-toc end -->

## Description

This folder contains the configuration needed to cross-compile the central
(cross-platform) components of espp for the following platforms:

* PC (Linux, MacOS, Windows)
  * C++
  * Python (through pybind 11)

Not all components of espp are cross-platform, so this library only references
the cross-platform components of espp. The cross-platform components are those
which do not depend on any specific hardware or platform.

Note: some components could be cross platform (e.g. various peripheral drivers
and such), but are not currently exposed via this library.

Some examples can be found in these folders:
- [../pc](../pc): This folder contains c++ various example code which uses the
  espp library.
- [../python](../python): This folder contains python code which uses the espp
  library.

All the examples in these folders require that the espp library is either
installed via pip or built first, as described below.

## Installing the python package with pip

The python bindings are packaged as the [`espp` package on
PyPI](https://pypi.org/project/espp/) (see
[README_PYPI.md](./README_PYPI.md)), built via scikit-build-core from the
[pyproject.toml](../pyproject.toml) at the repository root:

``` console
pip install espp
```

You can also install straight from git (requires CMake >= 3.20 and a C++20
compiler; pip clones the needed submodules automatically):

``` console
pip install git+https://github.com/esp-cpp/espp.git
```

When developing the bindings (or running the [../python](../python) example
scripts against local changes), use an editable install from the repository
root:

``` console
pip install -e .
```

Re-run it after changing C++ code to rebuild the extension (the CMake build
dir is persistent, so rebuilds are incremental); pure-python changes are
picked up automatically.

Wheels for Linux (x86_64 / aarch64), macOS (universal2), and Windows
(amd64) are built in CI by
[build_wheels.yml](../.github/workflows/build_wheels.yml) and published to
PyPI on each release.

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
* `./pc/espp/` - The `espp` python package (pure-python files, type stubs, and
  the compiled `espp._espp` pybind11 extension) - add `./pc` to your
  `PYTHONPATH` / `sys.path` to `import espp` from python code.

## Updating the python bindings

You should only need to regenerate / update the python bindings if the espp code
itself changes, and only if the changed code is exposed via this cross-platform
library - meaning it's part of the [./include/espp.hpp](./include/espp.hpp) or
otherwise pointed to by [espp.cmake](./espp.cmake).

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

**No manual editing of the generated file is required, and no build/compile step
is needed to fix it.** `autogenerate_bindings.py` strips C++20 `requires`-clauses
at parse time (so `Vector2d` and friends parse), then post-processes the
generated `pybind_espp.cpp` entirely with static string/regex passes
(`_postprocess_generated`) to fix every litgen/srcmlcpp bug that used to be fixed
by hand:

- bogus implicit default constructors (`Logger`, `Task`, the RTSP packetizers, ...),
- template-class nested args (`RangeMapper<int>`, `Bezier<espp::Vector2f>`, `Vector2d<float>`),
- static/instance method duplicates (`Task::get_id`),
- `std::shared_ptr` holders + bases (the RTP packetizer hierarchy, `JpegFrame`),
- unqualified RTSP nested types / nested-struct statics in named-ctor defaults
  (`espp::RtspClient::frame_callback_t`, `espp::RtspServer::Config::default_*`, ...),
- `def_readwrite` → `def_readonly` for non-copyable members (`RtspSession::Track` sockets),
- `py::call_guard<py::gil_scoped_release>()` on blocking methods (`Task::stop`,
  `Timer::cancel`, socket send/receive/accept, the RTSP request methods, ...).
  Without it, any method that joins the task thread or blocks on I/O deadlocks
  when called from python, because the blocked-on thread needs the GIL to invoke
  its python callback (see `_GIL_RELEASE_METHODS` in `autogenerate_bindings.py`;
  the hand-written `rtps` bindings apply the same guard).

The litgen dependency is unpinned and recent versions (0.20–0.22) regressed
nested-scope/template generation, which is why this automation is needed.

[`fix_generated_bindings.py`](./fix_generated_bindings.py) is kept only as an
optional diagnostic: if a future litgen version introduces *new* unqualified
names, configure the build with
`cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -S . -B build` and run it — it compiles
the generated file and reports clang's `'Y'; did you mean 'espp::X::Y'?`
suggestions so you can extend the static maps in `autogenerate_bindings.py`.

### Hand-written components (`cdr`, `rtps`)

`cdr` and `rtps` are **not** generated by litgen — they are bound by hand in
[./python_bindings/cdr_bindings.cpp](./python_bindings/cdr_bindings.cpp) and
[./python_bindings/rtps_bindings.cpp](./python_bindings/rtps_bindings.cpp)
(registered via `py_init_cdr` / `py_init_rtps` in `module.cpp`). litgen cannot
bind them usefully (output-reference reads, non-owning `std::span`,
`std::function` callbacks), and the hand-written shims give a clean, GIL-correct
Python API (`CdrWriter`/`CdrReader`, `RtpsParticipant` with
`publish(topic, bytes)` and `ReaderConfig.on_sample = callable(bytes)`). Edit
those files directly; regeneration never touches them.

