USB Device Component
====================

Overview
--------

``espp::UsbDevice`` is an idiomatic wrapper around ESP-IDF's ``esp_tinyusb``
managed component that assembles a **native USB device** from a *set of
selectable functions* on the ESP32-S3 / -S2 / -P4 USB-OTG peripheral, with a
**configurable VID/PID** and manufacturer / product / serial strings.

Today it can enable, in any combination (subject to the endpoint budget):

- A **CDC-ACM** function (virtual serial port).
- A **vendor-specific** function (``bInterfaceClass`` 0xFF, one bulk IN + one bulk
  OUT) that carries a raw byte stream and optionally advertises **WebUSB** + **MS
  OS 2.0** descriptors so a browser can talk to it driverlessly (and Windows binds
  WinUSB with no driver).
- A **HID** function (one interrupt IN, optionally one interrupt OUT) carrying an
  application-supplied report descriptor (for example a gamepad built with the
  espp ``hid-rp`` component), with input reports sent via ``write_hid_report()``.

Interface numbers, endpoint addresses and string indices are allocated
*sequentially* as functions are enabled, and the result is checked against the
USB-OTG endpoint budget (an error is reported via ``std::error_code`` if it is
exceeded). The model is designed so an **MSC** function can be added later
without changing the descriptor-building approach.

Because it uses the native USB-OTG peripheral rather than the built-in
USB-Serial-JTAG that carries the ESP console, a device can advertise its own USB
identifiers (for example ODrive-like ones) on a link that is fully separate from
the logging console.

``espp::UsbCdc`` is retained as a thin **CDC-only preset** over
``espp::UsbDevice`` for back-compatibility.

Features
--------

- Composable: enable a CDC function and/or a vendor/WebUSB function and/or a HID
  function (composite)
- Vendor-specific interface (class 0xFF) with a bulk IN + bulk OUT raw byte stream
- HID interface with an application-supplied report descriptor (built with
  ``hid-rp`` in the example) and ``write_hid_report()``
- WebUSB: BOS descriptor + WebUSB URL descriptor + MS OS 2.0 descriptor for
  driverless browser access, with a configurable landing-page URL
- Sequential interface / endpoint / string allocation with an endpoint-budget check
- Configurable VID, PID, and manufacturer / product / serial / interface strings
- No exceptions; ``initialize()`` reports failures via ``std::error_code``
- Safely marshals the TinyUSB RX callbacks (TinyUSB task context) into per-function
  user callbacks

Basic Usage
-----------

Composite CDC + vendor/WebUSB device, both interfaces carrying the same raw byte
stream:

.. code-block:: cpp

  espp::UsbDevice::Config cfg;
  cfg.vid = 0x1209; // pid.codes VID (ODrive uses this)
  cfg.pid = 0x0d32; // ODrive-like PID

  espp::UsbDevice::CdcFunction cdc;
  cdc.on_receive = [&](std::span<const uint8_t> data) { /* handle serial rx */ };
  cfg.cdc = cdc;

  espp::UsbDevice::VendorFunction vendor;
  vendor.webusb = true; // advertise WebUSB / MS OS 2.0 descriptors
  // landing_page_url defaults to the espp docs-hosted ODrive WebUSB console
  vendor.on_receive = [&](std::span<const uint8_t> data) { /* handle vendor rx */ };
  cfg.vendor = vendor;

  espp::UsbDevice usb(cfg);
  std::error_code ec;
  if (!usb.initialize(ec)) { /* handle ec (e.g. endpoint budget exceeded) */ }

  uint8_t hello[] = {'h', 'i', '\n'};
  usb.write_cdc(hello);
  usb.write_vendor(hello);

CDC-only preset (unchanged API):

.. code-block:: cpp

  espp::UsbCdc::Config cfg;
  cfg.vid = 0x1209;
  cfg.pid = 0x0d32;
  cfg.on_receive = [](std::span<const uint8_t> data) { /* handle rx */ };
  espp::UsbCdc usb(cfg);
  std::error_code ec;
  if (!usb.initialize(ec)) { /* handle ec */ }

Enabling the vendor / WebUSB class
----------------------------------

The vendor class is gated in ``esp_tinyusb`` behind a Kconfig option. To use the
vendor function you must set, in your project's ``sdkconfig.defaults`` (in
addition to the CDC options if you also enable CDC)::

  CONFIG_TINYUSB_CDC_ENABLED=y
  CONFIG_TINYUSB_CDC_COUNT=1
  CONFIG_TINYUSB_VENDOR_COUNT=1   # THE key enablement: compiles in the vendor class

Setting ``CONFIG_TINYUSB_VENDOR_COUNT`` greater than 0 makes ``esp_tinyusb``
define ``CFG_TUD_VENDOR`` and compile the TinyUSB vendor class driver. If the
vendor function is requested but ``CFG_TUD_VENDOR == 0``, ``initialize()`` fails
with ``std::errc::function_not_supported``. No custom ``tusb_config`` is required;
the BOS descriptor and the WebUSB / MS-OS-2.0 vendor control requests are provided
by ``espp::UsbDevice`` via the standard TinyUSB weak-callback overrides.

Enabling the HID class
----------------------

Like the vendor class, the HID class is gated in ``esp_tinyusb`` behind a Kconfig
option. To use the HID function you must set, in your project's
``sdkconfig.defaults``::

  CONFIG_TINYUSB_HID_COUNT=1   # compiles in the TinyUSB HID class driver (CFG_TUD_HID)

``espp::UsbDevice`` provides the required TinyUSB HID weak-callback overrides:
``tud_hid_descriptor_report_cb`` returns the stored report descriptor, while
``tud_hid_get_report_cb`` returns 0 and ``tud_hid_set_report_cb`` is a no-op since
the gamepad is input-only. Supply the report-descriptor bytes yourself (the
example builds them with the espp ``hid-rp`` component), assign them to
``HidFunction::report_descriptor``, and send input reports with
``write_hid_report(report_id, report)``. If the HID function is requested but
``CFG_TUD_HID == 0``, ``initialize()`` fails with
``std::errc::function_not_supported``.

Endpoint budget (ESP32-S3 USB-OTG)
----------------------------------

The ESP32-S3 (and -S2) USB-OTG core is full-speed and, besides the control
endpoint EP0, provides roughly **5 usable data IN endpoints** and **5 usable data
OUT endpoints**. Each function consumes:

.. list-table::
   :header-rows: 1

   * - Function
     - IN endpoints
     - OUT endpoints
   * - CDC-ACM
     - 2 (1 interrupt-IN notification + 1 bulk-IN)
     - 1 (bulk-OUT)
   * - Vendor / WebUSB
     - 1 (bulk-IN)
     - 1 (bulk-OUT)
   * - HID
     - 1 (interrupt-IN)
     - 0 or 1 (optional interrupt-OUT)
   * - MSC (future)
     - 1 (bulk-IN)
     - 1 (bulk-OUT)

This is why the device is **selectable** ("not all at once"). Combinations that
fit comfortably:

- CDC + Vendor: 3 IN / 2 OUT (used by the example)
- CDC + Vendor + HID: 4 IN / 2-3 OUT
- CDC + Vendor + MSC: 4 IN / 3 OUT

Enabling CDC + Vendor + HID + MSC together reaches 5 IN endpoints, which is at the
hard limit and is not recommended. ``espp::UsbDevice`` computes the totals as
functions are enabled and returns ``std::errc::value_too_large`` if the IN or OUT
budget is exceeded.

Extending with MSC
------------------

The **HID** function is implemented (see "Enabling the HID class" above): it
appends one HID interface (application-supplied report descriptor) claiming an
interrupt-IN endpoint, plus an optional interrupt-OUT endpoint.
``espp::UsbDevice::Config`` still reserves a ``std::optional`` slot for an
``MscFunction`` as a documented extension point; it is not implemented yet, and
enabling it today makes ``initialize()`` fail with
``std::errc::function_not_supported``. When implemented it slots into the same
sequential interface / endpoint / string allocator: an MSC function appends one
MSC interface (SCSI + storage read/write/capacity callbacks) claiming a bulk IN +
bulk OUT endpoint.

Notes
-----

- USB-OTG is only available on the ESP32-S2, ESP32-S3 and ESP32-P4 targets.
- Only one ``espp::UsbDevice`` / ``espp::UsbCdc`` instance may exist at a time
  (the TinyUSB stack and the BOS / vendor control callbacks are global).
- The receive callbacks run in the TinyUSB device task; keep them short and
  non-blocking. It is safe to call the matching ``write_*()`` from within them.
- The TinyUSB device lifecycle callbacks (``tud_mount_cb`` / ``tud_umount_cb`` /
  ``tud_suspend_cb`` / ``tud_resume_cb``) are owned by ``esp_tinyusb``. Register
  mount / unmount handlers via ``set_mount_callback()`` / ``set_unmount_callback()``
  rather than defining those callbacks yourself (which would be a duplicate
  symbol). On unmount the component clears the vendor + CDC TX FIFOs — so a
  departed host's queued backlog is not delivered to the next host that mounts —
  before invoking your callback; both handlers run in the TinyUSB device task.
- The WebUSB landing-page URL is configured *without* a scheme; the scheme is
  encoded separately via ``VendorFunction::url_scheme`` (0 = http, 1 = https).

.. ------------------------------- Example -------------------------------------

.. toctree::

   usb_cdc_example.md

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/usb_device.inc
.. include-build-file:: inc/usb_cdc.inc
