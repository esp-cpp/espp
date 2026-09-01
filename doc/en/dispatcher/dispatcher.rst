Dispatcher
**********

The `Dispatcher` routes framed messages from one byte stream to per-module
handlers, letting several independent protocols share a single USB vendor / CDC
/ socket / UART link. Rather than run a separate ``StreamParser`` per protocol
over the same bytes (each re-buffering the whole stream and needing its own
reset-on-overflow bookkeeping), a Dispatcher parses the
:doc:`../stream_frame/index` stream once and hands each complete frame to the
handler registered for its ``module`` id.

Module id
---------

The frame's ``module`` byte (0..255) is the routing key — a full byte, so up to
256 protocols can coexist on one stream. The message/transaction ``type`` and
the request/reply direction (``flags``) travel with the frame and are handed to
the module's handler untouched; the Dispatcher does not interpret them. espp
built-in protocols use, for example:

=========  ============
Module id  Protocol
=========  ============
0          OTA
4          crash dump
5          CAN bridge
=========  ============

A device-side dispatcher registers the modules it serves; frames for an
unregistered module — including the device's own replies echoed back (which
carry the reply flag) — are silently ignored. Application code may assign any
unused module id to its own protocol; nothing is hard-wired to a specific
service.

.. code-block:: cpp

   espp::Dispatcher dispatcher;
   dispatcher.register_module(0, [&](const espp::stream_frame::Frame &f) {
     // handle OTA frames: f.type, f.is_reply(), f.payload
   });
   dispatcher.register_module(4, [&](const espp::stream_frame::Frame &f) {
     // handle crash-dump frames
   });
   // feed raw received bytes; each complete frame is routed to its module
   usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) { dispatcher.feed(data); });

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/dispatcher.inc
