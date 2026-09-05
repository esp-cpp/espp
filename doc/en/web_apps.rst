Web Apps
********

espp ships a growing set of **self-contained browser tools** that talk directly
to your hardware using the Web Serial / WebUSB / WebHID APIs (Chromium-based
browsers) — nothing to install, no CDN, no network access. They are hosted
alongside this documentation:

    **→** `esp-cpp.github.io/espp/apps <https://esp-cpp.github.io/espp/apps/index.html>`_

Each app is a single HTML file; it also runs offline straight from its
``components/<name>/web/`` directory via a ``file://`` URL.

Device hub & dispatcher-module consoles
=======================================

Devices built on the :doc:`dispatcher <dispatcher/dispatcher>` / ``stream_frame``
protocol expose one or more *modules* over a single USB vendor (WebUSB)
interface. Start from the hub, which discovers what a device runs and links to
the matching console; each console also confirms its module is present when it
connects.

- **Device Hub** (``dispatcher_hub.html``) — connect over WebUSB / Web Serial,
  query the device's advertised modules, and open each module's console.
- **OTA Console** (``ota_console.html``) — stream a firmware ``.bin`` to the
  :doc:`ota <ota/ota>` component with live progress and a rollback-aware finish.
- **Core Dump Console** (``coredump_console.html``) — crash summary, ``core.elf``
  download, client-side nearest-symbol backtrace resolution, and erase for the
  :doc:`coredump <coredump/coredump>` service (over WebUSB or Web Serial, where
  it doubles as a serial monitor).
- **BLDC Haptics Console** (``haptics_console.html``) — live dial, detent
  presets, and control commands for the :doc:`bldc_haptics
  <haptics/bldc_haptics>` example.
- **CAN Bridge Console** (``can_bridge_console.html``) — configure the bus, send
  CAN frames, and watch a live monitor of received traffic for a USB↔CAN bridge.
- **DS402 Drive Panel** (``ds402_panel.html``) — in-browser CANopen SDO client
  plus a DS402 state machine / control panel for a :doc:`canopen
  <buses/canopen>` drive.
- **MCP266 Console** (``mcp266_console.html``) — status, motor, and configuration
  controls for the :doc:`mcp266 <motor_control/mcp266>` motor controller.

Motor control
=============

- **ODrive ASCII console** — interactive terminals with quick motor controls and
  live position / velocity plotting for the :doc:`odrive_ascii
  <motor_control/odrive_ascii>` protocol, over Web Serial
  (``odrive_console.html``) or WebUSB (``odrive_webusb_console.html``).
- **ODrive Native Control Panel** (``odrive_control_panel.html``) — endpoint-tree
  browser, live multi-signal plots, and typed read/write for the ODrive native
  (Fibre-endpoint) binary protocol.
- **WebHID Input Visualizer** (``hid_visualizer.html``) — decoded buttons /
  sticks / raw reports for any HID device, driven entirely by its report
  descriptor.

CAN & serial adapters
=====================

- **Basicmicro MCP Console** (``mcp_console.html``) — Web Serial console for the
  :doc:`Basicmicro <motor_control/basicmicro>` MCP packet-serial motor
  controllers.
- **CAN Bus Console** (``can_console.html``) — LAWICEL slcan serial monitor for a
  USB-CAN adapter (see :doc:`twai <buses/twai>`).

General
=======

- **Board Console & ESP Flasher** (``board_console.html``) — general-purpose Web
  Serial monitor with reset / bootloader controls and an ``esptool-js``-based
  firmware flasher.

Adding a new app
================

Any single-file app placed in a component's ``web/`` directory
(``components/<name>/web/*.html``, plus optional same-origin ``.js`` assets) is
hosted automatically by the docs workflow, and the `apps landing page
<https://esp-cpp.github.io/espp/apps/index.html>`_ lists it using the file's
``<title>`` and ``<meta name="description">`` tags — no registry to maintain.
Apps must be fully self-contained (no CDN resources) so they work offline and
under GitHub Pages' strict hosting.
