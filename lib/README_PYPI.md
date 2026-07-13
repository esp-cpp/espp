# espp

Python bindings for the cross-platform components of
[espp](https://github.com/esp-cpp/espp) — a collection of C++ components
originally built for ESP32 / ESP-IDF development, many of which are pure,
portable C++ and useful on the desktop as well.

The bindings are generated with [pybind11](https://github.com/pybind/pybind11)
and expose real native threads, sockets, and protocol implementations — unlike
Python threading, an `espp.Task` or `espp.Timer` runs on a real OS thread.

## Installation

```console
pip install espp
```

Or straight from the repository:

```console
pip install git+https://github.com/esp-cpp/espp.git
```

Building from source requires CMake (>= 3.20) and a C++20 compiler.

## What's included

- **Task / Timer**: native-threaded tasks and periodic timers with callbacks
- **Sockets**: UDP / TCP client & server helpers (unicast & multicast)
- **RTSP**: client & server with MJPEG / H.264 (de)packetizers
- **RTPS**: minimal RTPS participant for DDS interop (e.g. with embedded espp nodes)
- **CDR**: OMG Common Data Representation reader / writer
- **COBS**: consistent overhead byte stuffing encoder / decoder (incl. streaming)
- **Serialization**: alpaca-based binary serialization
- **Math**: vectors, Bézier curves, range mapping, fast approximations, filters (lowpass, Butterworth, ...), PID
- **Utilities**: logger, event manager, state machine, FTP, joystick, color, ...

The package ships complete type stubs (`py.typed`), so you get full
autocompletion and type checking in your editor.

## Quick example

```python
import time
import espp

def task_fn():
    print("hello from a native thread!")
    time.sleep(0.5)
    return False  # don't stop the task

task = espp.Task(task_fn, espp.Task.BaseConfig("example task"))
task.start()
time.sleep(2)
task.stop()
```

More example scripts live in the
[`python/`](https://github.com/esp-cpp/espp/tree/main/python) folder of the
repository.

## Links

- Documentation: https://esp-cpp.github.io/espp/
- Source & issues: https://github.com/esp-cpp/espp
