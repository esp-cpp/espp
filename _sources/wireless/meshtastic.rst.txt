Meshtastic
**********

The `MeshtasticNode` component is a minimal, radio-agnostic implementation of
the Meshtastic® over-the-air mesh protocol, sufficient to interoperate with
stock Meshtastic devices on a shared channel (the public "LongFast" channel by
default). It handles framing, AES-CTR encryption with the well-known default
channel key, sending and receiving text messages / node info / positions,
receive-side deduplication, and optional managed-flood rebroadcasting.

It is decoupled from any particular radio - you provide a transmit function
and feed it received frames - and pairs naturally with the :doc:`sx126x`
component.

.. note::

   This is an independent, clean-room implementation of the published
   Meshtastic protocol; it is not affiliated with or endorsed by Meshtastic
   LLC. "Meshtastic" is a registered trademark of Meshtastic LLC. See the
   component README for licensing and trademark details.

.. toctree::

   meshtastic_example

API Reference
-------------

.. include-build-file:: inc/meshtastic.inc
.. include-build-file:: inc/meshtastic_types.inc
.. include-build-file:: inc/meshtastic_protocol.inc
.. include-build-file:: inc/meshtastic_crypto.inc
.. include-build-file:: inc/meshtastic_protobuf.inc
