System Info, Control & Service
******************************

The `SystemInfo` class is a set of static getters over the corresponding
ESP-IDF calls: the chip model, revision, core count and feature flags, the
ESP-IDF version, the application description embedded in the image (project
name, version, build date and time, ELF SHA-256), the running and boot
partitions with the OTA image state, the reset reason, uptime, base MAC, flash
and PSRAM sizes, CPU frequency and the free / lowest-free heap.
``collect()`` gathers everything into one ``Snapshot`` and ``to_string()``
renders a boot-banner style summary.

The `SystemControl` class restarts the device: ``reboot()`` (``esp_restart``),
and ``reboot_to_bootloader()`` which sets the chip's *force download boot*
flag in its always-on register and restarts, so the next boot stays in the
ROM download mode instead of running the app — what holding the BOOT strap
during a reset does, without a button. The device then re-enumerates as the
ROM's own flashing interface (the USB CDC / DFU device on the ESP32-S2 / -S3
native USB port; USB-Serial-JTAG on the ESP32-C3 / -C6 / -H2 / -C5 / -C61 /
-H21 / -P4), ready for ``esptool`` / ``idf.py flash``. The classic ESP32 has
no software path (only the GPIO0 strap): ``bootloader_reboot_supported()`` is
false there and the call fails with ``operation_not_supported``. The
``*_after(delay)`` variants restart from a detached thread so a reply can
leave the transport first.

By default the reset tears the USB connection down and the ROM enumerates its
device afresh, which works for any application. On the ESP32-S2 / -S3 the ROM
can instead keep the USB peripheral's state across the reset
(``SystemControl::BootloaderOptions::usb_persist``, passed through as
``SystemService::Config::usb_persist``) so the host sees no re-plug. This is
**opt-in and off by default**: the ROM only expects it from an application
whose USB device is ROM-CDC/DFU-compatible (ESP-IDF's ROM USB console driver,
which is what performs the same sequence — ``usb_dc_prepare_persist()`` then
the persist flag — before its own reboot into the bootloader). A TinyUSB
composite device such as the espp examples' vendor + CDC has different
descriptors, and persisting it can leave the host with a stale enumeration
the bootloader cannot serve; leave the option off there.

The `SystemService` class serves both over **any byte stream** as a
:doc:`dispatcher <../dispatcher/dispatcher>` module (``espp.system`` v1,
module id 7 by default; ``Config::module`` moves an instance and hosts find
it through discovery by its protocol id). ``GET_INFO`` answers with a list of
tagged records (``[tag u8][len u8][value]``) a host decodes while skipping
tags it does not know, so fields can be added without a version bump. Every
reply echoes the request frame's correlation id, so a host that stamps its
requests can pair replies with them and drop stale ones.
``REBOOT`` and ``REBOOT_TO_BOOTLOADER`` reply ``OK`` first and restart after
the requested delay (clamped to ``Config::min_restart_delay``). Both are
guarded: ``Config::allow_reboot`` / ``allow_bootloader`` switch them off, the
optional ``on_reboot_request`` callback can veto a specific request (an
application with a motor running can refuse or defer), and the bootloader
restart is refused on chips without a software path. The ``INFO``
capabilities record tells a host up front which of the two it may offer.

The hosted `espp System Console
<https://esp-cpp.github.io/espp/apps/system_console.html>`_ web app speaks
the protocol over **WebUSB** (vendor interface) or **Web Serial** (CDC, where
it doubles as a serial monitor): a device-info panel, the two restart buttons
(with an in-page confirmation), and — when the device also advertises the
:doc:`monitor <../core/monitor>` component's ``MonitorService`` — heap-region
gauges and a live, sortable task table with a stream toggle.

.. image:: https://github.com/user-attachments/assets/bc5fa520-f3a0-40de-9b53-889a887c0322
   :alt: espp System Console: device info, reboot / bootloader controls and heap gauges
   :width: 100%
   :target: https://esp-cpp.github.io/espp/apps/system_console.html

.. image:: https://github.com/user-attachments/assets/c2c962bf-debb-44c7-96a7-95527218d953
   :alt: espp System Console streaming the task table (name, CPU %, stack high-water mark, priority, core)
   :width: 100%
   :target: https://esp-cpp.github.io/espp/apps/system_console.html

.. ------------------------------- Example -------------------------------------

.. toctree::

   system_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/system_info.inc
.. include-build-file:: inc/system_control.inc
.. include-build-file:: inc/system_service.inc
