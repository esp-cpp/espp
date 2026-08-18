"""High-level ODrive native client: channel, object tree, connect/find.

Usage::

    from espp_odrive import connect
    dev = connect("/dev/ttyUSB0")
    print(dev.vbus_voltage)                      # typed read
    dev.axis0.controller.input_pos = 3.14        # typed write
    dev.dump()                                   # pretty-print the live tree
"""

import json
import struct
import time

from .crc import PROTOCOL_VERSION, crc16
from .protocol import TYPE_CODECS, build_packet, parse_response
from .transport import SerialStreamTransport, Transport


class OdriveError(Exception):
    """Base class for all client errors."""


class TimeoutError_(OdriveError):
    """No response arrived before the deadline."""


class CanaryMismatch(OdriveError):
    """The device rejected our packet (json_crc / protocol-version mismatch)."""


# --------------------------------------------------------------------------- #
# Channel: sequence numbers + synchronous request/response over a Transport
# --------------------------------------------------------------------------- #
class Channel:
    """Owns the outbound sequence counter and the endpoint request/response loop.

    ``json_crc`` is the interface-definition canary; it is 0 until the JSON tree
    has been downloaded, which is fine because endpoint-0 reads use
    ``PROTOCOL_VERSION`` as their trailer.
    """

    def __init__(self, transport: Transport, timeout: float = 2.0):
        self._transport = transport
        self._timeout = timeout
        self._seq = 0
        self.json_crc = 0

    def endpoint_operation(self, endpoint_id: int, payload: bytes = b"",
                           expect_response: bool = True, output_len: int = 0) -> bytes:
        """Perform one endpoint read/write and return the response data bytes.

        Writes carry a non-empty ``payload``; reads set ``output_len`` to the
        number of bytes wanted back. A packet can do both at once.
        """
        if len(payload) >= 128:
            raise OdriveError("payload larger than 127 bytes is not supported")
        self._seq = (self._seq + 1) & 0x7FFF
        # One bit is hardwired to 1 to avoid clashing with the ASCII protocol,
        # mirroring the reference fibre client.
        seq = self._seq | 0x80
        packet = build_packet(seq, endpoint_id, output_len, payload,
                              expect_response, self.json_crc)
        self._transport.send_packet(packet)
        if not expect_response:
            return b""

        deadline = time.monotonic() + self._timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError_(
                    "no response for endpoint %d (seq %d) within %.1fs"
                    % (endpoint_id & 0x7FFF, seq, self._timeout))
            resp = self._transport.read_packet(remaining)
            if resp is None:
                continue
            parsed = parse_response(resp)
            if parsed is None:
                continue
            seq_no, data = parsed
            if (seq_no & 0x8000) and (seq_no & 0x7FFF) == (seq & 0x7FFF):
                return data
            # else: stale/unmatched response, keep waiting

    def read_endpoint_buffer(self, endpoint_id: int) -> bytes:
        """Read a long endpoint (e.g. endpoint 0 JSON) in chunks by offset."""
        buffer = b""
        while True:
            chunk = self.endpoint_operation(
                endpoint_id, struct.pack("<I", len(buffer)),
                expect_response=True, output_len=512)
            if not chunk:
                break
            buffer += chunk
        return buffer


# --------------------------------------------------------------------------- #
# Object tree
# --------------------------------------------------------------------------- #
class RemoteProperty:
    """A typed leaf endpoint. Reads/writes translate to endpoint operations."""

    def __init__(self, channel: Channel, name: str, endpoint_id: int,
                 type_name: str, access: str):
        self._channel = channel
        self._name = name
        self._id = endpoint_id
        self._type = type_name
        self._codec = TYPE_CODECS.get(type_name)
        self.can_read = "r" in access
        self.can_write = "w" in access

    def get_value(self):
        if self._codec is None:
            raise OdriveError("property %s has unsupported type %r" % (self._name, self._type))
        data = self._channel.endpoint_operation(
            self._id, b"", expect_response=True, output_len=self._codec.size)
        return self._codec.decode(data)

    def set_value(self, value):
        if self._codec is None:
            raise OdriveError("property %s has unsupported type %r" % (self._name, self._type))
        if not self.can_write:
            raise OdriveError("property %s is read-only" % self._name)
        buffer = self._codec.encode(value)
        # Wait for the ack (output_len 0) so a write raises on delivery failure.
        self._channel.endpoint_operation(
            self._id, buffer, expect_response=True, output_len=0)

    def _format_value(self):
        try:
            v = self.get_value()
        except OdriveError as e:
            return "<%s>" % e
        if self._name == "serial_number":
            return "0x%012X" % v
        if self._name == "error" or self._name.endswith("_error"):
            return "0x%X" % v
        return repr(v)


class RemoteObject:
    """A branch node exposing children (sub-objects, properties) as attributes."""

    def __init__(self, channel: Channel, name: str = ""):
        # Everything private goes through object.__setattr__ to avoid the
        # attribute-forwarding in __setattr__ below.
        object.__setattr__(self, "_channel", channel)
        object.__setattr__(self, "_name", name)
        object.__setattr__(self, "_children", {})  # name -> RemoteObject | RemoteProperty
        object.__setattr__(self, "_sealed", False)

    # -- tree construction --------------------------------------------------
    def _add(self, name, child):
        self._children[name] = child

    def _seal(self):
        object.__setattr__(self, "_sealed", True)

    # -- attribute access ---------------------------------------------------
    def __getattr__(self, name):
        # Only called when normal lookup fails (i.e. not a real attribute).
        children = object.__getattribute__(self, "_children")
        if name in children:
            child = children[name]
            if isinstance(child, RemoteProperty):
                if not child.can_read:
                    # __getattr__ must raise AttributeError (never a custom
                    # exception): a write-only property has no readable value,
                    # and hasattr()/getattr() default handling relies on this.
                    raise AttributeError("property %s is write-only" % name)
                return child.get_value()
            return child
        raise AttributeError(name)

    def __setattr__(self, name, value):
        children = object.__getattribute__(self, "_children")
        child = children.get(name)
        if isinstance(child, RemoteProperty):
            child.set_value(value)
            return
        if isinstance(child, RemoteObject):
            raise OdriveError("cannot assign to sub-object %s" % name)
        object.__setattr__(self, name, value)

    def __dir__(self):
        return sorted(set(list(super().__dir__()) + list(self._children.keys())))

    # -- introspection ------------------------------------------------------
    def get_property(self, name) -> RemoteProperty:
        """Return the underlying :class:`RemoteProperty` (no read triggered)."""
        child = self._children.get(name)
        if not isinstance(child, RemoteProperty):
            raise KeyError(name)
        return child

    def _dump_lines(self, indent, out):
        for key, child in self._children.items():
            if isinstance(child, RemoteObject):
                out.append("%s%s:" % (indent, key))
                child._dump_lines(indent + "  ", out)
            else:
                out.append("%s%s = %s (%s)" % (indent, key, child._format_value(), child._type))


class Device(RemoteObject):
    """The root object returned by :func:`connect` / :func:`find`.

    In addition to the attribute tree it holds the transport, the raw JSON
    descriptor, and the computed ``json_crc``.
    """

    def __init__(self, channel: Channel, transport: Transport,
                 json_bytes: bytes, json_data):
        super().__init__(channel, name="")
        object.__setattr__(self, "_transport", transport)
        object.__setattr__(self, "_json_bytes", json_bytes)
        object.__setattr__(self, "_json_data", json_data)
        object.__setattr__(self, "json_crc", channel.json_crc)

    # -- convenience path get/set ------------------------------------------
    def get(self, path: str):
        """Typed read of a dotted path, e.g. ``dev.get("axis0.error")``."""
        return self._resolve(path).get_value()

    def set(self, path: str, value):
        """Typed write of a dotted path, e.g. ``dev.set("axis0.controller.input_pos", 1.0)``."""
        self._resolve(path).set_value(value)

    def _resolve(self, path: str) -> RemoteProperty:
        obj = self
        parts = path.split(".")
        for p in parts[:-1]:
            obj = object.__getattribute__(obj, "_children")[p]
        prop = object.__getattribute__(obj, "_children")[parts[-1]]
        if not isinstance(prop, RemoteProperty):
            raise KeyError("%s is not a property" % path)
        return prop

    def dump(self) -> str:
        """Pretty-print the whole tree with live values; returns the string too."""
        out = []
        self._dump_lines("", out)
        text = "\n".join(out)
        print(text)
        return text

    def close(self):
        self._transport.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# --------------------------------------------------------------------------- #
# Tree building from the endpoint-0 JSON descriptor
# --------------------------------------------------------------------------- #
def _build_tree(channel: Channel, parent: RemoteObject, members):
    for m in members:
        name = m.get("name")
        if name is None:
            continue
        type_str = m.get("type")
        if type_str == "object":
            child = RemoteObject(channel, name=name)
            _build_tree(channel, child, m.get("members", []))
            child._seal()
            parent._add(name, child)
        elif type_str == "function":
            # Functions are out of scope for this client; skip cleanly.
            continue
        elif type_str is not None:
            ep_id = m.get("id")
            if ep_id is None:
                continue
            parent._add(name, RemoteProperty(
                channel, name, int(ep_id), type_str, m.get("access", "r")))


# --------------------------------------------------------------------------- #
# Entry points
# --------------------------------------------------------------------------- #
def _device_from_transport(transport: Transport, timeout: float = 2.0) -> Device:
    channel = Channel(transport, timeout=timeout)
    # Endpoint 0 (JSON descriptor) reads use PROTOCOL_VERSION as the trailer, so
    # this works before json_crc is known.
    json_bytes = channel.read_endpoint_buffer(0)
    if not json_bytes:
        raise OdriveError("device returned an empty endpoint-0 JSON descriptor")
    try:
        json_str = json_bytes.decode("ascii")
    except UnicodeDecodeError as e:
        raise OdriveError("endpoint-0 descriptor is not ASCII: %r" % e)
    json_data = json.loads(json_str)
    # The endpoint canary is CRC16 over the exact JSON bytes, seeded with
    # PROTOCOL_VERSION (NOT the 0x1337 stream init).
    channel.json_crc = crc16(json_bytes, PROTOCOL_VERSION)

    device = Device(channel, transport, json_bytes, json_data)
    _build_tree(channel, device, json_data)
    device._seal()
    return device


def connect(port: str, baudrate: int = 115200, timeout: float = 2.0) -> Device:
    """Connect to an ODrive over a serial/UART port and return a :class:`Device`.

    Downloads the endpoint-0 JSON descriptor, computes ``json_crc``, and builds
    the object tree.
    """
    transport = SerialStreamTransport(port, baudrate=baudrate)
    try:
        return _device_from_transport(transport, timeout=timeout)
    except Exception:
        transport.close()
        raise


def connect_transport(transport: Transport, timeout: float = 2.0) -> Device:
    """Like :func:`connect`, but over an already-constructed :class:`Transport`.

    This is the seam for a future USB-bulk backend: build the transport, then
    hand it here.
    """
    return _device_from_transport(transport, timeout=timeout)


def find(timeout: float = 5.0, baudrate: int = 115200):
    """Scan available serial ports and return the first ODrive that answers.

    Returns a :class:`Device` or ``None`` if none was found within ``timeout``.
    """
    from serial.tools import list_ports

    deadline = time.monotonic() + timeout
    tried = set()
    while time.monotonic() < deadline:
        for info in list_ports.comports():
            if info.device in tried:
                continue
            tried.add(info.device)
            try:
                return connect(info.device, baudrate=baudrate,
                               timeout=min(2.0, max(0.5, deadline - time.monotonic())))
            except Exception:
                continue
        time.sleep(0.2)
    return None
