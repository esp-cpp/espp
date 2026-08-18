"""Transport abstraction for the ODrive native protocol.

A :class:`Transport` moves whole *packets* between the client and the device.
Two concrete shapes exist:

* :class:`SerialStreamTransport` -- the UART/serial link. Fibre has no packet
  structure on a raw serial line, so every packet is wrapped in a small stream
  frame with a CRC8 header and a CRC16 trailer (see ``odrive_native_stream.hpp``).
  This is what the device shim speaks over its PTY.
* a future USB-bulk backend -- over USB each bulk transfer already *is* one
  packet, so it would send/recv raw packets with no framing. The seam is the
  :class:`Transport` interface below: implement ``send_packet``/``read_packet``
  and the rest of the stack (channel, object tree) is unchanged.
"""

import struct
import time
from abc import ABC, abstractmethod

from .crc import CRC16_INIT, crc8, crc16

SYNC_BYTE = 0xAA
MAX_PACKET_SIZE = 128  # stream framing caps a packet at 127 bytes (len < 128)


# --------------------------------------------------------------------------- #
# Stream framing (serial backend)
# --------------------------------------------------------------------------- #
def stream_frame(packet: bytes) -> bytes:
    """Wrap one packet in a fibre serial stream frame.

    ``[0xAA][len u8][crc8(sync,len) init 0x42][packet][crc16(packet) init 0x1337, big-endian]``
    """
    if len(packet) >= MAX_PACKET_SIZE:
        raise ValueError("packet larger than 127 bytes is not supported by stream framing")
    header = bytes([SYNC_BYTE, len(packet)])
    header += bytes([crc8(header)])
    trailer = struct.pack(">H", crc16(packet, CRC16_INIT))  # big-endian
    return header + packet + trailer


class StreamDeframer:
    """Stateful deframer: feed received bytes, get back complete packets.

    Resynchronizes on the ``0xAA`` sync byte and validates both CRCs; a frame
    that fails either CRC (or carries ``len >= 128``) is dropped and the
    deframer hunts for the next sync byte.
    """

    def __init__(self):
        self._buf = bytearray()

    def push(self, data: bytes):
        self._buf.extend(data)
        packets = []
        while True:
            # Resync to the first sync byte.
            sync = self._buf.find(SYNC_BYTE)
            if sync < 0:
                self._buf.clear()
                break
            if sync > 0:
                del self._buf[:sync]
            if len(self._buf) < 3:
                break
            length = self._buf[1]
            if length >= MAX_PACKET_SIZE or crc8(bytes(self._buf[:3])) != 0:
                # Bad header: drop the sync byte and hunt for the next one.
                del self._buf[0]
                continue
            frame_len = 3 + length + 2
            if len(self._buf) < frame_len:
                break  # wait for more bytes
            packet = bytes(self._buf[3:3 + length])
            got = struct.unpack(">H", bytes(self._buf[3 + length:3 + length + 2]))[0]
            if crc16(packet, CRC16_INIT) != got:
                del self._buf[0]
                continue
            packets.append(packet)
            del self._buf[:frame_len]
        return packets


# --------------------------------------------------------------------------- #
# Transport interface + serial backend
# --------------------------------------------------------------------------- #
class Transport(ABC):
    """Moves whole packets to/from the device."""

    @abstractmethod
    def send_packet(self, packet: bytes) -> None:
        raise NotImplementedError

    @abstractmethod
    def read_packet(self, timeout: float):
        """Return one received packet, or ``None`` if none arrived in ``timeout``."""
        raise NotImplementedError

    def close(self) -> None:
        # Default no-op; subclasses with a real resource override this.
        return

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class SerialStreamTransport(Transport):
    """Serial/UART backend: stream-frames outgoing packets and deframes input."""

    def __init__(self, port: str, baudrate: int = 115200, serial_obj=None):
        if serial_obj is not None:
            self._serial = serial_obj
        else:
            import serial  # pyserial; imported lazily so USB-only users need not install it
            self._serial = serial.Serial(port, baudrate, timeout=0)
        self._deframer = StreamDeframer()
        self._pending = []

    def send_packet(self, packet: bytes) -> None:
        self._serial.write(stream_frame(packet))
        self._serial.flush()

    def read_packet(self, timeout: float):
        if self._pending:
            return self._pending.pop(0)
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            data = self._serial.read(256)
            if data:
                new = self._deframer.push(data)
                if new:
                    self._pending.extend(new)
                    return self._pending.pop(0)
            if time.monotonic() >= deadline:
                return None
            if not data:
                time.sleep(0.001)

    def close(self) -> None:
        try:
            self._serial.close()
        except Exception:
            # Best-effort close: the port may already be gone (device
            # unplugged) or never fully opened; nothing useful to do on error.
            pass
