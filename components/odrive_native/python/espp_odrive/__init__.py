"""espp_odrive -- a clean, dependency-light Python client for the ODrive
legacy *native* (Fibre endpoint) protocol.

This is an odrivetool-equivalent re-implementation of the documented wire
protocol (see ``PROTOCOL.md``). It depends only on the Python standard library
plus ``pyserial`` for the serial backend -- there is **no** dependency on the
``odrive``/``fibre`` pip package.

Quick start::

    from espp_odrive import connect
    dev = connect("/dev/ttyUSB0")
    print("vbus:", dev.vbus_voltage)
    dev.axis0.controller.input_pos = 3.14
    dev.dump()
"""

from .ascii import OdriveAscii
from .crc import PROTOCOL_VERSION, crc8, crc16
from .device import (
    CanaryMismatch,
    Channel,
    Device,
    OdriveError,
    RemoteObject,
    RemoteProperty,
    TimeoutError_,
    connect,
    connect_transport,
    find,
)
from .protocol import TYPE_CODECS, build_packet, parse_response
from .transport import (
    SerialStreamTransport,
    StreamDeframer,
    Transport,
    stream_frame,
)

__version__ = "0.1.0"

__all__ = [
    "connect",
    "connect_transport",
    "find",
    "Device",
    "RemoteObject",
    "RemoteProperty",
    "Channel",
    "OdriveError",
    "TimeoutError_",
    "CanaryMismatch",
    "Transport",
    "SerialStreamTransport",
    "StreamDeframer",
    "stream_frame",
    "TYPE_CODECS",
    "build_packet",
    "parse_response",
    "crc8",
    "crc16",
    "PROTOCOL_VERSION",
    "OdriveAscii",
    "__version__",
]
