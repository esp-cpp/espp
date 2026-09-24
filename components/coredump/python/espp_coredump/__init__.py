"""espp_coredump — pure-Python host tool to read an espp device's core dump over USB.

Speaks the espp ``stream_frame`` framing + core-dump stream protocol
(dispatcher module 4) over the device's USB vendor (WebUSB) interface — the
same protocol ``components/coredump/web/coredump_console.html`` implements in
the browser and the ``coredump`` example serves on-device — and hands the
downloaded ``core.elf`` to ESP-IDF's ``esp-coredump`` decoder.

The codec (:mod:`espp_coredump.frame`) and protocol
(:mod:`espp_coredump.protocol`) are standard-library only; the USB transport
(:mod:`espp_coredump.transport`) needs `pyusb`, imported lazily.

Typical use::

    from espp_coredump import CoreDumpClient, UsbVendorTransport, extract_elf
    with UsbVendorTransport() as t:
        image = CoreDumpClient(t, progress=lambda r, tot: ...).read_image()
    open("core.elf", "wb").write(extract_elf(image))
"""

from .client import CoreDumpClient
from .elf import extract_elf, find_elf_offset
from .protocol import (CoreDumpError, DataInfo, DiscoveryInfo, ErrorInfo, MessageType,
                       ModuleInfo)
from .transport import DEFAULT_PID, DEFAULT_VID, TransportError, UsbVendorTransport

__all__ = [
    "CoreDumpClient",
    "UsbVendorTransport",
    "TransportError",
    "CoreDumpError",
    "MessageType",
    "ErrorInfo",
    "DataInfo",
    "DiscoveryInfo",
    "ModuleInfo",
    "extract_elf",
    "find_elf_offset",
    "DEFAULT_VID",
    "DEFAULT_PID",
]

__version__ = "0.1.0"
