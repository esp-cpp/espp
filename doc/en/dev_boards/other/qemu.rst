QEMU (emulator)
***************

Qemu
----

The `espp::Qemu` component is a board-support package for `Espressif's QEMU
<https://github.com/espressif/qemu>`_: not a board, but the few virtual
peripherals the emulator offers, wrapped with the same shape as the real-board
BSPs so an espp application runs with no hardware at all. It builds for the
three targets the emulator supports (``esp32``, ``esp32c3``, ``esp32s3``).

- **Display**: the virtual RGB panel as an LVGL display
  (``initialize_display()``). What LVGL draws appears in the QEMU window, or
  off screen in a headless run (``SDL_VIDEODRIVER=dummy``), and can be captured
  with the QEMU monitor's ``screendump``. The display needs a display backend:
  with ``-display none`` the panel driver's first flush waits forever.
- **Ethernet**: the OpenCores Ethernet MAC (``initialize_ethernet()``), a real
  lwIP interface on QEMU's user-mode NAT or on a tap device. Needs
  ``CONFIG_ETH_USE_OPENETH``.
- **SD card** (ESP32 machine only): the SD/MMC host (``initialize_sdcard()``),
  a FAT volume on the image file passed with ``-drive file=...,if=sd``.

Everything else (touch, I2C, I2S, Wi-Fi, BLE, USB) is not emulated. See
:doc:`../../qemu` for installing the emulator, headless runs, screenshots,
networking and debugging.

.. ------------------------------- Example -------------------------------------

.. toctree::

   qemu_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/qemu.inc
