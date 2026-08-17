USB CDC Transport Component
===========================

Overview
--------

``espp::UsbCdc`` is a thin, idiomatic wrapper around ESP-IDF's ``esp_tinyusb``
managed component. It presents a single dedicated **native USB CDC-ACM** (virtual
serial port) interface on the ESP32-S3 / -S2 / -P4 USB-OTG peripheral, with a
**configurable VID/PID** and manufacturer / product / serial strings.

Because it uses the native USB-OTG peripheral rather than the built-in
USB-Serial-JTAG that carries the ESP console, a device can advertise its own USB
identifiers (for example ODrive-like ones) on a link that is fully separate from
the logging console. This also lays the groundwork for adding a WebUSB *vendor*
interface later.

Features
--------

- Native USB CDC-ACM interface over USB-OTG, separate from the log console
- Configurable VID, PID, and manufacturer / product / serial / interface strings
- Byte-stream transport: receive callback for RX, ``write()`` for TX
- No exceptions; ``initialize()`` reports failures via ``std::error_code``
- Safely marshals the TinyUSB RX callback (TinyUSB task context) into the user callback

Basic Usage
-----------

.. code-block:: cpp

  espp::UsbCdc::Config cfg;
  cfg.vid = 0x1209; // pid.codes VID (ODrive uses this)
  cfg.pid = 0x0d32; // ODrive-like PID
  cfg.manufacturer = "espp";
  cfg.product = "espp USB CDC";
  cfg.on_receive = [](std::span<const uint8_t> data) { /* handle rx */ };

  espp::UsbCdc usb(cfg);
  std::error_code ec;
  if (!usb.initialize(ec)) { /* handle ec */ }
  uint8_t hello[] = {'h', 'i', '\n'};
  usb.write(hello);

Notes
-----

- USB-OTG is only available on the ESP32-S2, ESP32-S3 and ESP32-P4 targets.
- Enable ``CONFIG_TINYUSB_CDC_ENABLED=y`` in your project.
- Only one ``espp::UsbCdc`` instance may drive a given CDC port.
- The receive callback runs in the TinyUSB device task; keep it short and non-blocking.

.. ------------------------------- Example -------------------------------------

.. toctree::

   usb_cdc_example.md

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/usb_cdc.inc
