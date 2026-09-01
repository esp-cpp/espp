Stream Frame
************

The `stream_frame` component provides a minimal wire framing (v2) for carrying
typed, length-delimited, CRC-verified messages over any raw byte stream. It is
intentionally free of any ESP-IDF / FreeRTOS dependency, so the framing (CRC-32,
frame building, incremental parsing with resynchronization) builds and
unit-tests on a host with nothing more than a C++20 standard library.

Wire format (all multi-byte fields little-endian)::

    [magic u16 = 0x4F54 ("OT")][flags u8][module u8][type u8][len u32][payload][crc32 u32]

- ``magic`` — the u16 ``0x4F54`` ("OT"), so the raw bytes are ``0x54`` ('T')
  then ``0x4F`` ('O').
- ``flags`` — bit0 = reply (0 = request host→device, 1 = reply/event
  device→host); bits 1–3 reserved; bits 4–7 = format version (currently 1). The
  reserved bits + version leave room to extend the payload semantics later.
- ``module`` — routing / protocol id (0..255). :doc:`../dispatcher/index` routes
  on this, so up to 256 protocols can share one stream.
- ``type`` — message / transaction type within the module (0..255). The
  ``Transaction`` enum gives recommended standard values (Write / Read /
  WriteRead / Custom); a protocol may otherwise define its own type values and
  carry a finer opcode in the payload.
- ``len`` — payload length, ``<= kMaxPayloadSize`` (4096). The parser rejects
  and resynchronizes past any oversized length, bounding memory usage.
- ``crc32`` — standard zlib CRC-32 over ``magic..payload`` (golden check value
  ``crc32("123456789") == 0xCBF43926``).

``build_frame()`` encodes a frame; ``StreamParser::feed()`` consumes arbitrary
chunks (frames may be split or batched) and yields **every** complete,
CRC-verified frame, resynchronizing at the next intact frame after any
corruption. It does not filter by module or type — routing a multi-protocol
stream (and ignoring unknown modules) is the job of :doc:`../dispatcher/index`.
The buffering is bounded because the length field is capped.

.. ------------------------------- Example -------------------------------------

.. literalinclude:: ../../../components/stream_frame/example/main/stream_frame_example.cpp
   :language: cpp
   :start-after: //! [stream_frame example]
   :end-before: //! [stream_frame example]

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/stream_frame.inc
