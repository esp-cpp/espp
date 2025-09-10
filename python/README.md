# ESPP Python Library

This folder contains some test python scripts for loading the `espp` shared
objects built within the `espp/lib` folder into Python.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [ESPP Python Library](#espp-python-library)
  - [Scripts](#scripts)
  - [Setup](#setup)
    - [Install Python Requirements](#install-python-requirements)
  - [Quick Start](#quick-start)

<!-- markdown-toc end -->

## Scripts

This section gives a brief overview of what the scripts in this folder do.

- `task.py`: This script is a simple example of how to use the `espp` library to
  create a task. It demonstrates how to create a task, run it, and handle its
  results. Note: unlike python multithreading, this task is actually run in a
  separate thread.
- `timer.py`: This script demonstrates how to use the `espp` library to create a
  timer. It shows how to create a timer, start it, and handle its expiration.
  The timer runs in a separate thread and calls a callback function when it
  expires. Note: unlike python multithreading, this timer is actually run in a
  separate thread.
- `udp_client.py` and `udp_server.py`: These scripts demonstrate how to use the
  `espp` library to create a UDP client and server. The server listens for
  incoming UDP packets and prints them to the console, while the client sends
  UDP packets to the server.
- `rtsp_client.py` and `rtsp_server.py`: These scripts demonstrate how to use the
  `espp` library to create an RTSP client and server. The server streams video
  from a webcam or display capture, while the client receives the stream and
  displays it in a window. Note: these scripts require additional 3rd party
  libraries such as `opencv`, `mss`, and `zeroconf`.
- `cobs_test.py`: This script demonstrates the COBS (Consistent Overhead Byte Stuffing)
  component functionality through Python bindings using pointer-free tests. The test
  verifies class instantiation, buffer management, data extraction methods, thread safety,
  and method availability. Note: Full COBS functionality (encoding/decoding with data)
  requires raw pointer support which has limitations in the current Python bindings.
  

## Setup

For all scripts, you must have the `espp` shared objects built and
accessible in the `espp/lib` folder. See the [espp/lib](../lib/README.md)
README for more details.

### Install Python Requirements

Some tests (e.g. `rtsp_client.py`, `rtsp_server.py`) make use of 3rd party
libraries such as `zeroconf, opencv, mss` to facilitate mDNS discovery, image
display / webcam capture, and display capture. For these tests, you will need to
install the `requirements.txt`:

```console
# create the virtual environment
python3 -m venv env
# activate it
source env/bin/activate
# install the requirements
pip install -r requirements.txt
```

Afterwards, you only need to source the environment and run the test in
question:

```console
source env/bin/activate
# now you can run the tests
python3 rtsp_server.py
```

## Quick Start

To run a test, you can use the following commands:

```console
python3 <test_name>.py
# e.g.
python3 task.py
# or 
python3 udp_client.py
``` 

Note: the `udp_client.py` script requires a running instance of the
`udp_server.py` script. To run the server, use the following command from
another terminal:

```console
python3 udp_server.py
```

