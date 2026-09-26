"""espp_ota — pure-Python host tool to OTA-update an espp device over USB.

Speaks the espp ``stream_frame`` framing + OTA stream protocol (dispatcher
module 0) over the device's USB vendor (WebUSB) interface — the same protocol
``components/ota/web/ota_console.html`` implements in the browser and the
``ota`` example serves on-device.

The codec (:mod:`espp_ota.frame`) and protocol (:mod:`espp_ota.protocol`) are
standard-library only; the USB transport (:mod:`espp_ota.transport`) needs
`pyusb`, imported lazily.

Typical use::

    from espp_ota import OtaClient, UsbVendorTransport
    with UsbVendorTransport() as t:
        OtaClient(t, progress=lambda w, tot: ...).flash(open("app.bin", "rb").read())
"""

from .client import OtaClient
from .protocol import ErrorInfo, MessageType, OtaError, ProgressInfo
from .transport import DEFAULT_PID, DEFAULT_VID, TransportError, UsbVendorTransport

__all__ = [
    "OtaClient",
    "UsbVendorTransport",
    "TransportError",
    "OtaError",
    "MessageType",
    "ErrorInfo",
    "ProgressInfo",
    "DEFAULT_VID",
    "DEFAULT_PID",
]

# The version is the espp wheel's (one source of truth: the package metadata
# setuptools-scm derives from the git tag). Run from the source tree without
# the wheel installed, there is no metadata, and the version says so.
try:
    from importlib.metadata import PackageNotFoundError, version as _dist_version

    try:
        __version__ = _dist_version("espp")
    except PackageNotFoundError:
        __version__ = "0.0.0+source"
except ImportError:  # pragma: no cover - Python < 3.8
    __version__ = "0.0.0+source"
