Stream Frame
************

The `stream_frame` component provides a minimal wire framing for carrying typed,
length-delimited, CRC-verified messages over any raw byte stream. It is
intentionally free of any ESP-IDF / FreeRTOS dependency, so the framing (CRC-32,
frame building, incremental parsing with resynchronization) builds and
unit-tests on a host with nothing more than a C++20 standard library.

Wire format (all multi-byte fields little-endian)::

    [magic u16 = 0x4F54 ("OT")][type u8][len u32][payload: len bytes][crc32 u32]

- ``magic`` — the u16 ``0x4F54`` ("OT"), so the raw bytes are ``0x54`` ('T')
  then ``0x4F`` ('O').
- ``type`` — an application message-type byte. Each protocol owns a disjoint
  range of the byte space; parsers ignore types they do not recognize, so
  multiple protocols coexist on one stream (see :doc:`../dispatcher/index`).
- ``len`` — payload length, ``<= kMaxPayloadSize`` (4096). The parser rejects
  and resynchronizes past any oversized length, bounding memory usage.
- ``crc32`` — standard zlib CRC-32 over ``magic..payload`` (golden check value
  ``crc32("123456789") == 0xCBF43926``).

``build_frame()`` encodes a frame; ``StreamParser::feed()`` consumes arbitrary
chunks (frames may be split or batched) and yields the complete, CRC-verified
frames, resynchronizing at the next intact frame after any corruption. The
buffering is bounded because the length field is capped.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/stream_frame.inc
