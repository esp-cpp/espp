"""ODrive legacy (Fibre) CRC primitives.

Both widths use the same bit-by-bit, **non-reflected, MSB-first** algorithm.
This is a clean re-implementation of the documented wire algorithm (see
``PROTOCOL.md``); it does NOT depend on the ``odrive``/``fibre`` pip package.

Constants (from the fw-v0.5.1 reference):

* CRC8  -- poly ``0x37``,   init ``0x42``   (UART *stream* framing header only)
* CRC16 -- poly ``0x3d65``, init ``0x1337`` (stream framing + endpoint-0 trailer)
* the endpoint ``json_crc`` canary is CRC16 seeded with ``PROTOCOL_VERSION`` (1),
  **not** ``0x1337``.
"""

CRC8_POLY = 0x37
CRC8_INIT = 0x42
CRC16_POLY = 0x3D65
CRC16_INIT = 0x1337
PROTOCOL_VERSION = 1


def calc_crc(remainder: int, value: int, poly: int, bitwidth: int) -> int:
    """Fold a single byte ``value`` through the running ``remainder``."""
    topbit = 1 << (bitwidth - 1)
    mask = (1 << bitwidth) - 1
    remainder ^= (value << (bitwidth - 8)) & mask
    for _ in range(8):
        if remainder & topbit:
            remainder = ((remainder << 1) ^ poly) & mask
        else:
            remainder = (remainder << 1) & mask
    return remainder & mask


def crc8(data: bytes, init: int = CRC8_INIT) -> int:
    """CRC8 over ``data`` (poly 0x37)."""
    rem = init
    for b in data:
        rem = calc_crc(rem, b, CRC8_POLY, 8)
    return rem


def crc16(data: bytes, init: int = CRC16_INIT) -> int:
    """CRC16 over ``data`` (poly 0x3d65).

    Use ``init=CRC16_INIT`` (0x1337) for stream framing, or
    ``init=PROTOCOL_VERSION`` (1) for the endpoint ``json_crc`` canary.
    """
    rem = init
    for b in data:
        rem = calc_crc(rem, b, CRC16_POLY, 16)
    return rem
