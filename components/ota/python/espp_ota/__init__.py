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

__version__ = "0.1.0"
