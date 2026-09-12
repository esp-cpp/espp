USB Host Component
==================

Overview
--------

``espp::UsbHost`` is the host-side counterpart to ``espp::UsbDevice``. It drives
the ESP32-S2 / -S3 / -P4 USB-OTG peripheral as a **USB host**, enumerates
attached devices, and exposes the **HID** class devices it finds — mice,
keyboards, gamepads, and vendor-specific HID devices (for example another ESP
running ``espp::UsbDevice`` as a HID device, such as an
``espp::WdiUsbPeripheral``).

It is a thin, idiomatic wrapper over the ESP-IDF USB Host library (``usb``) and
the ``usb_host_hid`` class driver: it owns the whole host lifecycle — installing
the host library and HID driver, running their event tasks, opening interfaces,
and teardown — and marshals the driver's C callbacks into per-device
``std::function`` s. Like the rest of espp it does not throw and reports failures
via ``std::error_code``.

Report directions are named from the connected **device's** point of view, as in
the USB HID spec: an *Input* report is device→host (delivered to a ``HidDevice``
input callback), an *Output* report is host→device (sent with
``HidDevice::send_output_report()``). This mirrors ``espp::UsbDevice`` exactly,
so the two ends of a link (for example the device and host roles of the ``wdi``
component) line up.

Features
--------

- Installs / uninstalls the USB Host library and the HID class driver and runs
  their event-handling tasks.
- Device **connect / disconnect** callbacks, with an optional filter predicate so
  only the devices you care about are opened (by VID/PID, interface, etc.).
- Per-device **Input report** callback (device→host) delivering the raw report
  bytes (report id in byte 0 for report-ID'd descriptors).
- Send **Output reports** (host→device) and issue the HID class control requests
  (Get/Set Report, Get/Set Idle, Set Protocol).
- Read a connected device's **HID report descriptor**.
- No exceptions; ``initialize()`` reports failures via ``std::error_code``.

Basic Usage
-----------

.. code-block:: cpp

  espp::UsbHost host({
      .on_device_connected =
          [](const std::shared_ptr<espp::UsbHost::HidDevice> &dev) {
            auto info = dev->info();
            printf("connected: %s %s %04x:%04x\n", info.manufacturer.c_str(),
                   info.product.c_str(), info.vid, info.pid);
            dev->set_input_callback([](std::span<const uint8_t> report) {
              // handle a device->host Input report (report[0] is the report id)
            });
          },
      .on_device_disconnected = [](const auto &) { /* ... */ },
      // optional: only open the devices you want
      // .should_open = [](const auto &info, const auto &) { return info.vid == 0x1209; },
  });

  std::error_code ec;
  if (!host.initialize(ec)) { /* handle ec */ }

  // later, send an Output report (host->device):
  std::array<uint8_t, 4> payload{/* ... */};
  host.devices().front()->send_output_report(/*report_id*/ 0x02, payload, ec);

Requirements and caveats
------------------------

- USB-OTG **host** mode is only available on the **ESP32-S2, -S3 and -P4**.
- Only one ``espp::UsbHost`` may exist at a time (the USB Host library and HID
  class driver are global singletons). It cannot coexist with ``espp::UsbDevice``
  (they both claim the USB-OTG peripheral).
- The board must be able to source **VBUS** to the attached device — a board with
  a USB-A host port / VBUS switch, or a self-powered hub. ``UsbHost`` does not
  manage board power.
- On the ESP32-S3 the USB-Serial-JTAG shares the USB-OTG PHY, so when the host
  role is active the **console must run on UART0** (see the example's
  ``sdkconfig.defaults``).
- The ``usb`` and ``usb_host_hid`` components come from the ESP Component
  Registry via the IDF component manager. On ESP-IDF ≥ 6.0 ``usb_host_hid``
  declares its ``usb`` dependency only through the manager, so build with the
  component manager **on** (the default) rather than the manager-off flow used by
  the device-side USB examples.

Roadmap
-------

Only the **HID** class driver is wired up today (it covers mice, keyboards,
gamepads and vendor HID devices, and is what the ``wdi`` host role needs). The
component is structured so other class drivers (CDC-ACM, MSC) can be layered in
later without changing the host-lifecycle model — the same way
``espp::UsbDevice`` composes CDC / Vendor / HID functions on the device side.

.. ------------------------------- Example -------------------------------------

.. toctree::

   usb_host_example.md

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/usb_host.inc
