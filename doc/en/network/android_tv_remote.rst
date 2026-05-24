Android TV Remote
*****************

The ``AndroidTvRemote`` component provides a practical Android TV Remote v2 client
for Google Chromecast and Google TV devices.

This first release focuses on:

- discovery via mDNS ``_googlecast._tcp.local``
- pairing over TLS on port ``6467``
- control over TLS on port ``6466``
- persistent client certificate and key storage in NVS
- remote keys, IME text input, and media control helpers

It is intentionally a pragmatic subset of Android TV Remote v2 rather than a
complete protocol implementation.

Notes:

- pairing requires correct handling of the 6-character pairing code shown on the TV
- client credentials must persist across reboots so the paired relationship survives
- some devices do not expose every command surface consistently

.. ------------------------------- Example -------------------------------------

.. toctree::

   android_tv_remote_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/android_tv_remote.inc
