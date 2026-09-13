Wheelchair Digital Interface (WDI)
**********************************

The ``wdi`` component implements the `Open-Mobility-Hub Wheelchair HID
<https://open-mobility-hub.github.io/wheelchair-digital-interface/>`_
specification (v3.2) — a standard interface that lets an accessory (special
switches, an alternative joystick, a phone app, a companion MCU) drive a powered
wheelchair and receive status/telemetry back, over **USB** or **Bluetooth LE**.

The component is layered so the same protocol serves every combination of role
and transport:

- **Protocol core** (``include/detail/wdi_protocol.hpp``) — host-testable and
  ESP-free: the five HID reports (Control, Feedback, Request-Feedback, Keepalive,
  Keepalive-Response), their bitfields, and pack/parse helpers.
- **HID report descriptor** (``include/wdi_hid.hpp``) — the vendor (usage page
  0xFF00) report descriptor, built with the espp ``hid-rp`` component. Only the
  USB HID transport needs it (BLE carries the same reports as GATT
  characteristics).
- **Device role** — the app / accessory: sends Control, receives Feedback.

  - ``espp::WdiDevice`` (``wdi.hpp``): the transport-agnostic core with the app's
    keepalive state machine.
  - ``espp::WdiBlePeripheral`` (``wdi_ble.hpp``): the WDI GATT service on
    ``ble_gatt_server``.
  - ``espp::WdiUsbPeripheral`` (``wdi_usb.hpp``): the WDI HID descriptor on
    ``espp::UsbDevice``.
- **Host role** — the wheelchair: receives Control, sends Feedback, and runs the
  keepalive **watchdog** (drive-disable if the accessory goes quiet).

  - ``espp::WdiHost`` (``wdi_host.hpp``): the transport-agnostic core with the
    host's keepalive watchdog.
  - ``espp::WdiBleCentral`` (``wdi_ble_central.hpp``): a NimBLE central that
    connects to a WDI peripheral.
  - ``espp::WdiUsbHost`` (``wdi_usb_host.hpp``): an ``espp::UsbHost`` (USB Host
    HID) that talks to a WDI HID device.

Report directions are named from the **device** (accessory) point of view — an
*Input* report is device→host (Control / Request-Feedback / Keepalive), an
*Output* report is host→device (Feedback / Keepalive-Response). All payloads are
little-endian **except** the 128-bit Host UUID, which is big-endian per the spec.

Keepalive / timeout
===================

The app sends a Control / Request-Feedback / Keepalive report every ~233 ms; the
host's window is 257 ms and it disconnects + **drive-disables** after 3
consecutive missed windows. ``WdiDevice::poll()`` emits a keepalive when one is
due; ``WdiHost::poll()`` fires the disconnect callback when the watchdog expires.
Both take an injectable clock, so both cores are unit-tested on a host
(``test/wdi_device_host_test.cpp``, ``test/wdi_host_host_test.cpp``).

Safety
======

This component can **emulate** a WDI device or host for development and testing.
A powered wheelchair is safety-critical: do not connect an emulator to a real
chair without the manufacturer's guidance, and honor the keepalive / drive-disable
semantics — a lost link must drop to a safe, stopped state.

.. ------------------------------- Examples ------------------------------------

.. toctree::

   ../../../components/wdi/README.md

Examples
========

- ``components/wdi/ble_example`` — the **device** role over BLE (advertises the
  WDI service and drives a wheelchair).
- ``components/wdi/usb_example`` — the **device** role over USB (enumerates as a
  WDI HID device).
- ``components/wdi/ble_central_example`` — the **host** role over BLE (scans for
  and connects to a WDI peripheral).
- ``components/wdi/usb_host_example`` — the **host** role over USB (enumerates a
  WDI HID device from the host side).

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/wdi_protocol.inc
.. include-build-file:: inc/wdi.inc
.. include-build-file:: inc/wdi_hid.inc
.. include-build-file:: inc/wdi_ble.inc
.. include-build-file:: inc/wdi_usb.inc
.. include-build-file:: inc/wdi_host.inc
.. include-build-file:: inc/wdi_usb_host.inc
.. include-build-file:: inc/wdi_ble_central.inc
