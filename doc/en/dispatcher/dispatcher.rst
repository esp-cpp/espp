Dispatcher
**********

The `Dispatcher` routes framed messages from one byte stream to per-module
handlers, letting several independent protocols share a single USB vendor / CDC
/ socket / UART link. Rather than run a separate ``StreamParser`` per protocol
over the same bytes (each re-buffering the whole stream and needing its own
reset-on-overflow bookkeeping), a Dispatcher parses the
:doc:`../stream_frame/index` stream once and hands each complete frame to the
handler registered for its module id.

Module id convention
--------------------

The module id is the high nibble of the message-type byte (``type >> 4``). espp
built-in protocols place their request opcodes so each protocol occupies one
high nibble, and use bit 7 to mark device→host replies:

===========  =========  ==============  =============
Module id    Protocol   Requests        Replies
===========  =========  ==============  =============
0            OTA        ``0x0X``        ``0x8X``
4            crash dump ``0x4X``        ``0xCX``
5            CAN bridge ``0x5X``        ``0xDX`` *(example)*
===========  =========  ==============  =============

A device-side dispatcher registers the request modules (0, 4, 5, ...). It never
receives the reply-typed frames (high nibble 8..15); if one arrives it lands on
an unregistered module and is ignored, so requests and replies of the same
protocol can never be confused. Application code may assign any unused module id
to its own protocol — nothing is hard-wired to a specific service. Frames for an
unregistered module are silently ignored.

.. code-block:: cpp

   espp::Dispatcher dispatcher;
   dispatcher.register_module(0, [&](uint8_t type, std::span<const uint8_t> payload) {
     // handle OTA frames
   });
   dispatcher.register_module(4, [&](uint8_t type, std::span<const uint8_t> payload) {
     // handle crash-dump frames
   });
   // feed raw received bytes; each complete frame is routed to its module
   usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) { dispatcher.feed(data); });

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/dispatcher.inc
