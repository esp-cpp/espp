Stream Frame
************

The `stream_frame` component provides a minimal wire framing (v2) for carrying
typed, length-delimited, CRC-verified messages over any raw byte stream. It is
intentionally free of any ESP-IDF / FreeRTOS dependency, so the framing (CRC-32,
frame building, incremental parsing with resynchronization) builds and
unit-tests on a host with nothing more than a C++20 standard library.

Wire format (all multi-byte fields little-endian)::

    [magic u16 = 0x4F54 ("OT")][flags u8][module u8][type u8]
        {[correlation u16] iff flags bit1}[len u32][payload][crc32 u32]

- ``magic`` — the u16 ``0x4F54`` ("OT"), so the raw bytes are ``0x54`` ('T')
  then ``0x4F`` ('O').
- ``flags`` — **bit0 = reply**: the frame's direction in a request/response
  exchange — ``0`` = request (initiator→responder), ``1`` = response/event
  (responder→initiator). Which side is the "initiator" depends on the protocol's
  roles. **bit1 = correlation present**; bits 2–3 reserved for future optional
  header fields; bits 4–7 = format version (currently 1).
- ``module`` — routing / protocol id (0..255). :doc:`../dispatcher/index` routes
  on this, so up to 256 protocols can share one stream.
- ``type`` — message / transaction type within the module (0..255). The
  ``Transaction`` enum gives recommended standard values (Write / Read /
  WriteRead / Custom); a protocol may otherwise define its own type values and
  carry a finer opcode in the payload.
- ``correlation`` — **optional** u16 (present only when ``flags`` bit1 is set): a
  protocol-defined correlation / sequence id (opaque to the codec) for matching a
  response to its request when more than one may be outstanding. It lives in the
  header, covered by the CRC. Optional fields are flag-gated, so a frame without
  them is byte-identical to before and more can be added under bits 2–3 without
  another breaking change.
- ``len`` — payload length, ``<= kMaxPayloadSize`` (4096). The parser rejects
  and resynchronizes past any oversized length, bounding memory usage.
- ``crc32`` — standard zlib CRC-32 over the whole header + payload (golden check
  value ``crc32("123456789") == 0xCBF43926``).

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
