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
- An **X-Input** function that presents the device as a wired **Xbox 360
  controller** (a custom TinyUSB application class driver built into this
  component — no ``CFG_TUD_*`` count needed). Gamepad state is sent with
  ``update_xinput_state()`` (see ``xinput.hpp``) and rumble/LED reports arrive via an
  ``on_rumble`` callback. Because the host's XUSB driver only binds a recognized
  Xbox 360 VID/PID and the built-in vendor class also claims interface class 0xFF,
  **use X-Input as the only enabled function** (Microsoft's IDs, for emulation /
  testing of your own device only).

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
- X-Input interface (wired Xbox 360 controller) via a custom application class
  driver, with ``update_xinput_state()`` and an ``on_rumble`` callback
- Console over CDC: optionally route the ESP console (stdout) to the CDC interface
  (``CdcFunction::route_console`` or ``route_console_to_cdc()``) so one native USB
  cable carries the logs alongside a vendor / HID / XInput interface; non-blocking,
  and teed to the primary UART console by default
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
``tud_hid_descriptor_report_cb`` returns the stored report descriptor and
``tud_hid_get_report_cb`` returns 0. Supply the report-descriptor bytes yourself
(the example builds them with the espp ``hid-rp`` component), assign them to
``HidFunction::report_descriptor``, and send input reports with
``write_hid_report(report_id, report)``.

To **receive** host→device OUTPUT / SET_REPORT reports (for request/response HID
protocols such as the Nintendo Switch Pro controller handshake), set
``HidFunction::on_receive`` (or ``set_hid_receive_callback()``) and set
``HidFunction::has_out_endpoint`` for interrupt-OUT reports. The callback is
invoked from the TinyUSB task with the report id as byte 0 of its span; reply by
sending an INPUT report with ``write_hid_report()``. If the HID function is
requested but ``CFG_TUD_HID == 0``, ``initialize()`` fails with
``std::errc::function_not_supported``.

Enabling X-Input (Xbox 360)
---------------------------

X-Input needs **no** ``CFG_TUD_*`` count — it is served by a custom TinyUSB
application class driver built into this component (registered via the weak
``usbd_app_driver_get_cb``, forced into the link with ``-u``). An X-Input-only
project therefore enables no built-in USB class; the ``xinput_example`` disables
them all (``CONFIG_TINYUSB_CDC_ENABLED=n``). Keep ``CFG_TUD_VENDOR`` at 0 so the
built-in bulk vendor driver does not claim the X-Input 0xFF interface, and use
X-Input as the **only** enabled function (it then advertises the Xbox 360 identity
+ ``0xFF/0xFF/0xFF`` device class so the host's XUSB driver binds it). Send gamepad
state with ``update_xinput_state()`` and receive rumble/LED via ``on_rumble``. The
interface uses one interrupt-IN (0x81) + one interrupt-OUT endpoint with separate
endpoint numbers, and the report DMA buffers are word-aligned as the ESP32-S3 DWC2
requires.

Routing the console over CDC
----------------------------

When the native USB port is given to TinyUSB for a vendor / HID / XInput interface,
the ESP console can no longer live on USB-Serial-JTAG (on the ESP32-S3 it shares
the USB-OTG PHY, so it contends and reboot-loops the device). Add a CDC function
and route the console to it, and one native USB cable carries both the logs and the
other interface:

.. code-block:: cpp

   espp::UsbDevice::CdcFunction cdc;
   cdc.route_console = true;   // redirect stdout -> CDC at the end of initialize()
   // cdc.tee_console = true;  // (default) also keep the primary UART console
   usb_cfg.cdc = cdc;
   usb_cfg.vendor = my_vendor; // CDC is just the log channel
   espp::UsbDevice usb(usb_cfg);
   usb.initialize(ec);         // console now on CDC (teed to UART)

Or call ``usb.route_console_to_cdc()`` yourself after a successful ``initialize()``.
``printf`` / ``ESP_LOG`` / ``espp::Logger`` all write to ``stdout``, which is
``freopen``ed onto a tiny write-only VFS device; its writes forward to
``write_cdc()`` only when the whole chunk fits the TX FIFO (never blocking on an
absent reader, and not gated on DTR) and, with ``tee_console`` (default), are also
written to the primary UART console so ``idf.py monitor`` keeps working. Recommended
console config: UART0 primary (``CONFIG_ESP_CONSOLE_UART_DEFAULT``) with
USB-Serial-JTAG as the secondary console for early-boot logs. The ``ota`` example
uses this.

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
   * - X-Input (Xbox 360)
     - 1 (interrupt-IN)
     - 1 (interrupt-OUT)
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
   xinput_example.md

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/usb_device.inc
.. include-build-file:: inc/usb_cdc.inc
.. include-build-file:: inc/xinput.inc
