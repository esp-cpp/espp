COBS (Consistent Overhead Byte Stuffing)
****************************************

The `COBS` component provides efficient encoding and decoding for the Consistent
Overhead Byte Stuffing algorithm, a packet framing protocol that ensures reliable
packet boundaries in communication streams.

Key features:
* Zero-byte elimination with consistent overhead (max ⌈n/254⌉ + 1 bytes)
* Single packet API for basic encoding/decoding
* Streaming API with thread-safe encoder/decoder classes
* Support for fragmented data streams and packet batching

The component is designed for embedded communication protocols where reliable
packet framing is essential, such as UART, SPI, or other serial interfaces.

.. ------------------------------- Example -------------------------------------

.. toctree::

   cobs_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/cobs.inc
