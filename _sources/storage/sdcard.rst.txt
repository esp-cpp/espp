SD Card
*******

The ``espp::SdCard`` component brings up an SD / microSD card over **SDSPI** (any
target) or the **SDMMC (SDIO)** peripheral (ESP32, ESP32-S3, ESP32-P4), and keeps
card initialization and FAT mounting as two separate steps.

ESP-IDF's convenience functions (``esp_vfs_fat_sdspi_mount()`` /
``esp_vfs_fat_sdmmc_mount()``) probe the card and mount its FAT volume in one
call, and own the card for as long as it is mounted. ``SdCard`` instead probes the
card in ``initialize()`` and mounts / unmounts the volume with ``mount()`` /
``unmount()``, so the same card can be handed to another user -- a USB host
through ``espp::UsbDevice``'s MSC function -- and taken back without re-probing it.

Interface configuration
-----------------------

``Config::interface`` is a ``std::variant`` of:

- ``SpiConfig``: the SPI host and chip-select pin; optionally the bus pins, when
  the component should initialize (and later free) the bus rather than share one
  the application or BSP already owns.
- ``SdmmcConfig``: the slot, 1- or 4-bit width, the pins (routed through the
  GPIO matrix on targets that support it), the bus clock, and an optional on-chip
  LDO channel that powers the card (the ESP32-P4 feeds its SD pads from LDO
  channel 4).

Both share the mount settings: ``mount_point``, ``mount_on_initialize``,
``format_if_mount_failed`` (off by default; ``format()`` is explicit),
``max_files``, ``allocation_unit_size`` and ``disk_status_check``.

Handing the card to a USB host
------------------------------

Initialize with ``mount_on_initialize = false`` (or ``unmount()`` first) and pass
``card()`` as ``espp::UsbDevice::MscMedium::sd_card``; the MSC function then mounts
the card at its own path while the application owns it and unmounts it while the
PC has it. See the ``usb_device`` component's ``msc_example`` and the SD card
example README.

.. ------------------------------- Example -------------------------------------

.. toctree::

   sdcard_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/sdcard.inc
