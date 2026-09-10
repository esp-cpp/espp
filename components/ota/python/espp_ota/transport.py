"""USB vendor-interface transport for the OTA host tool.

Talks to the device's WebUSB / vendor interface (``bInterfaceClass == 0xFF``,
one bulk IN + one bulk OUT endpoint) — the same interface ``ota_console.html``
uses from the browser. Uses `pyusb` (libusb); it is imported lazily so the
:mod:`espp_ota.frame` / :mod:`espp_ota.protocol` layers stay stdlib-only.

Default device id is the espp ``UsbDevice`` default, ``0x1209:0x0d32``; both are
overridable (a project may set its own VID/PID).
"""

from __future__ import annotations

from typing import List, Optional, Tuple

DEFAULT_VID = 0x1209
DEFAULT_PID = 0x0D32
VENDOR_CLASS = 0xFF


class TransportError(RuntimeError):
    pass


def _import_usb():
    try:
        import usb.core  # noqa: F401
        import usb.util  # noqa: F401
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise TransportError(
            "pyusb is required for the USB transport. Install it with "
            "`pip install pyusb` (and a libusb backend: `brew install libusb` on "
            "macOS, `apt install libusb-1.0-0` on Linux; on Windows bind WinUSB "
            "with Zadig if the device is not already driverless)."
        ) from exc
    import usb.core as core
    import usb.util as util

    return core, util


def list_devices(vid: int = DEFAULT_VID, pid: Optional[int] = None) -> List[Tuple[int, int, str]]:
    """Return (vid, pid, description) for candidate devices matching the filter."""
    core, util = _import_usb()
    kwargs = {"find_all": True, "idVendor": vid}
    if pid is not None:
        kwargs["idProduct"] = pid
    out: List[Tuple[int, int, str]] = []
    for dev in core.find(**kwargs):
        try:
            desc = util.get_string(dev, dev.iProduct) or ""
        except Exception:
            desc = ""
        out.append((dev.idVendor, dev.idProduct, desc))
    return out


class UsbVendorTransport:
    """Open the vendor bulk pipe of an espp device and read/write raw frames.

    Use as a context manager::

        with UsbVendorTransport() as t:
            t.write(frame_bytes)
            reply = t.read(4111, timeout_ms=5000)
    """

    def __init__(
        self,
        vid: int = DEFAULT_VID,
        pid: Optional[int] = DEFAULT_PID,
        serial: Optional[str] = None,
        interface: Optional[int] = None,
    ) -> None:
        self._vid = vid
        self._pid = pid
        self._serial = serial
        self._want_itf = interface
        self._core, self._util = _import_usb()
        self._dev = None
        self._itf_num: Optional[int] = None
        self._ep_in = None
        self._ep_out = None
        self._claimed = False

    # -- lifecycle ------------------------------------------------------------
    def open(self) -> "UsbVendorTransport":
        core, util = self._core, self._util

        def _match(dev):
            if self._serial is None:
                return True
            try:
                return util.get_string(dev, dev.iSerialNumber) == self._serial
            except Exception:
                return False

        kwargs = {"idVendor": self._vid}
        if self._pid is not None:
            kwargs["idProduct"] = self._pid
        dev = core.find(custom_match=_match, **kwargs)
        if dev is None:
            raise TransportError(
                f"no device found matching vid=0x{self._vid:04x}"
                + (f" pid=0x{self._pid:04x}" if self._pid is not None else "")
                + (f" serial={self._serial!r}" if self._serial else "")
            )
        self._dev = dev

        cfg = dev.get_active_configuration()
        itf, ep_in, ep_out = self._find_vendor_interface(cfg)
        self._itf_num = itf.bInterfaceNumber
        self._ep_in, self._ep_out = ep_in, ep_out

        # Detach a kernel driver if one grabbed the interface (rare for a pure
        # vendor class, but be safe on Linux).
        try:
            if dev.is_kernel_driver_active(self._itf_num):
                dev.detach_kernel_driver(self._itf_num)
        except (NotImplementedError, self._core.USBError):
            pass  # no kernel driver bound (or the platform can't detach) -> nothing to do

        self._util.claim_interface(dev, self._itf_num)
        self._claimed = True
        return self

    def _find_vendor_interface(self, cfg):
        util = self._util
        for itf in cfg:
            if self._want_itf is not None and itf.bInterfaceNumber != self._want_itf:
                continue
            if itf.bInterfaceClass != VENDOR_CLASS:
                continue
            ep_in = ep_out = None
            for ep in itf:
                is_bulk = util.endpoint_type(ep.bmAttributes) == util.ENDPOINT_TYPE_BULK
                if not is_bulk:
                    continue
                if util.endpoint_direction(ep.bEndpointAddress) == util.ENDPOINT_IN:
                    ep_in = ep
                else:
                    ep_out = ep
            if ep_in is not None and ep_out is not None:
                return itf, ep_in, ep_out
        raise TransportError(
            "no vendor interface (class 0xFF with a bulk IN+OUT endpoint pair) found; "
            "is the device running the OTA example over its WebUSB/vendor interface?"
        )

    def close(self) -> None:
        if self._dev is None:
            return
        try:
            if self._claimed and self._itf_num is not None:
                self._util.release_interface(self._dev, self._itf_num)
        except Exception:
            pass  # teardown is best-effort (device may already be gone/unplugged)
        try:
            self._util.dispose_resources(self._dev)
        except Exception:
            pass  # ditto: free libusb handles best-effort, never raise from close()
        self._dev = None
        self._claimed = False

    def __enter__(self) -> "UsbVendorTransport":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    # -- I/O ------------------------------------------------------------------
    def write(self, data: bytes, timeout_ms: int = 5000) -> None:
        n = self._ep_out.write(data, timeout_ms)
        if n != len(data):
            raise TransportError(f"short write: {n}/{len(data)} bytes")

    def read(self, max_len: int, timeout_ms: int = 5000) -> bytes:
        """Read up to ``max_len`` bytes; return ``b""`` on timeout (not an error)."""
        try:
            arr = self._ep_in.read(max_len, timeout_ms)
        except self._core.USBError as exc:
            # A genuine timeout is expected (poll again), but a real I/O error must
            # propagate. pyusb>=1.1 raises the USBTimeoutError subclass; older pyusb
            # raises USBError with errno 110 (ETIMEDOUT). Do NOT treat an unknown
            # errno as a timeout — that would silently swallow backend failures.
            timeout_cls = getattr(self._core, "USBTimeoutError", None)
            is_timeout = (timeout_cls is not None and isinstance(exc, timeout_cls)) or (
                getattr(exc, "errno", None) == 110)
            if is_timeout:
                return b""
            raise
        return bytes(arr)

    @property
    def description(self) -> str:
        if self._dev is None:
            return "<closed>"
        return f"0x{self._dev.idVendor:04x}:0x{self._dev.idProduct:04x} (interface {self._itf_num})"
