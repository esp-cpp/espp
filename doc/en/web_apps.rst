Web Apps
********

espp ships a growing set of **self-contained browser tools** that talk directly
to your hardware using the Web Serial / WebUSB / WebHID APIs (Chromium-based
browsers) — nothing to install, no CDN, no network access. They are hosted
alongside this documentation:

    **→** `esp-cpp.github.io/espp/apps <https://esp-cpp.github.io/espp/apps/index.html>`_

Each app is a single HTML file; it also runs offline straight from its
``components/<name>/web/`` directory via a ``file://`` URL.

The landing page groups the apps by category (*device management*, *motor
control*, *bus tools*, *input devices*, *utilities*), can be filtered by name,
description, protocol id, transport or category, and sorted by name or
category. Each card shows the espp dispatcher protocols the app speaks (a
trailing ``?`` marks an optional one, e.g. the System Console's
``espp.monitor:1?``) and the browser transports it uses (WebUSB, Web Serial,
WebHID). The same table is published next to the apps as ``registry.json`` and
``registry.js`` (``window.ESPP_APPS``) — the `app registry`_ the Device Hub
uses to link every app that can drive a device.

Device hub & dispatcher-module consoles
=======================================

Devices built on the :doc:`dispatcher <dispatcher/dispatcher>` / ``stream_frame``
protocol expose one or more *modules* over a single USB vendor (WebUSB)
interface. Start from the hub, which discovers what a device runs and links to
the matching console (passing the module id along as ``?module=N``); each
console also runs discovery itself when it connects and talks to whichever
module advertises its protocol id (``espp.ota``, ``espp.coredump``, ...), so a
device may serve a protocol on any dispatcher module id.

- **Device Hub** (``dispatcher_hub.html``) — connect over WebUSB / Web Serial,
  query the device's advertised modules, and open each module's console. With
  the hosted `app registry`_ beside it, it also lists every app whose
  protocols the device advertises ("Apps for this device") and, per module,
  the other apps that speak its protocol ("Also works with") — e.g. a CAN
  bridge advertises the CAN Bridge Console, and the DS402 Drive Panel is
  offered too. Opening an app hands the device off: the hub releases it and
  the app connects to it on load (see `Auto-connect, auto-reconnect and
  hand-off`_).
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
  plus a DS402 state machine / control panel for a
  :doc:`canopen <buses/canopen>` drive, with an object-dictionary browser (the
  device's stored EDS, an EDS file, or the built-in CiA 301/402 table, scanned
  over SDO).
- **MCP266 Console** (``mcp266_console.html``) — status, motor, and configuration
  controls for the :doc:`mcp266 <motor_control/mcp266>` motor controller.
- **System Console** (``system_console.html``) — device info (chip, firmware,
  partitions, reset reason, uptime, memory), reboot and reboot-into-bootloader
  for the :doc:`system <system/system>` component, plus live heap gauges and a
  task table when the device serves the :doc:`monitor <core/monitor>`
  component's ``MonitorService``.

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

Data plotting
=============

- **Serial Plotter** (``telemetry.html``) — auto-parses columnar serial
  output (a header line followed by matching numeric rows), discards everything
  that does not fit the detected schema, and re-evaluates when a new header
  arrives. It plots a high number of points efficiently (``uPlot``) with drag
  zoom, a per-series legend, and cursor readout, and saves or loads the
  capture as CSV. Optional **2D X–Y** and **3D X–Y–Z** modes plot the same parsed
  columns against each other. It can also plot **binary telemetry over WebUSB**
  from an espp device running :doc:`espp::Telemetry <telemetry/telemetry>`
  (typed float channels, device timestamps).

.. image:: https://github.com/user-attachments/assets/64668e83-8ff8-4ba8-9f97-1be5e1c3ad6d
   :alt: espp Serial Plotter plotting a Lorenz-attractor capture as a time series
   :width: 100%
   :target: https://esp-cpp.github.io/espp/apps/telemetry.html

General
=======

- **Board Console & ESP Flasher** (``board_console.html``) — general-purpose Web
  Serial monitor with reset / bootloader controls and an ``esptool-js``-based
  firmware flasher.

Adding a new app
================

Any single-file app placed in a component's ``web/`` directory
(``components/<name>/web/*.html``, plus optional same-origin ``.js`` assets) is
hosted automatically by the docs workflow and listed on the `apps landing page
<https://esp-cpp.github.io/espp/apps/index.html>`_ — there is no
hand-maintained list; the page describes itself with tags in its ``<head>``:

.. code-block:: html

   <title>espp System Console (WebUSB / Web Serial)</title>
   <meta name="description" content="Device info, reboot, ... for the espp system component.">
   <meta name="espp-category" content="device management">
   <meta name="espp-protocols" content="espp.system:1 espp.monitor:1?">
   <meta name="espp-transports" content="webusb webserial">

- ``<title>`` / ``description`` — the card's title and blurb.
- ``espp-category`` (**required**) — one of ``device management``,
  ``motor control``, ``bus tools``, ``input devices``, ``utilities``; the
  section the card is listed under.
- ``espp-protocols`` — space-separated ``id:version`` entries naming the
  espp dispatcher protocols the app speaks (the same ids a device advertises
  in discovery, see :doc:`dispatcher/custom_modules`); a trailing ``?`` marks
  a protocol the app can do without. Omit the tag for an app that speaks no
  espp protocol (a plain serial console, a WebHID tool).
- ``espp-transports`` — space-separated subset of ``webusb``, ``webserial``,
  ``webhid``.

The docs build (``doc/generate_apps_index.py``) fails with a clear message on
a page without a category, an unknown category or transport, or a malformed
protocol entry. Apps must be fully self-contained (no CDN resources) so they
work offline and under GitHub Pages' strict hosting.

.. _app registry:

The app registry
----------------

Besides ``index.html`` the generator writes ``registry.json`` and
``registry.js`` next to the apps: one record per app —
``{file, title, description, category, protocols: [{id, version, optional}],
transports}`` — built from the tags above. ``registry.js`` assigns the object
to ``window.ESPP_APPS``; the Device Hub loads it as an optional sibling script
(``<script src="registry.js">``). When it is present the hub links, for a
discovered device, every app whose *required* protocols the device advertises
(each link carries the id of the module that speaks the app's protocol, as
``?module=N``), and per module the other apps that speak its protocol; it
notes a protocol version the app does not implement (``app speaks v1, device
advertises v2``) and listed protocols the device lacks (``optional
espp.monitor not advertised``). Without the registry (``file://``, or a copy of
the hub on its own) the hub behaves as before and links only the app each
module advertises. Devices that send a version-1 discovery payload (no
protocol ids) are also linked that way, since nothing can be matched.

``node components/dispatcher/web/test/apps_registry_test.js`` checks every
page's metadata, the generator's outputs and failure modes, and the hub's
matching rules (it is not run in CI — run it after touching an app's
``<head>``, the generator, or the hub).

Auto-connect, auto-reconnect and hand-off
-----------------------------------------

WebUSB and Web Serial remember, per origin and device, which devices a page
was granted; a page may list them (``navigator.usb.getDevices()`` /
``navigator.serial.getPorts()``) and open one again without the chooser. The
dispatcher-module consoles build on that:

- **Auto-connect.** A console opened with
  ``?autoconnect=1&transport=usb|serial&vid=0x1209&pid=0x0d32[&serial=...]``
  (besides ``?module=N``) connects on load to the permitted device those ids
  name — no chooser, no click. Ids are matched on vid + pid and, for WebUSB,
  the serial number when both sides report one; with no match the console
  says so and waits for Connect (a page cannot open the chooser without a
  click).
- **Hand-off from the hub.** Every app link the hub renders carries those
  parameters for the connected device. Only one page can hold a device
  (WebUSB claims the vendor interface, Web Serial opens the port), so clicking
  a link makes the hub *close its connection first*, then open the app, which
  auto-connects. The hub then shows a "Device handed off" banner with a
  **Reconnect** button; it never reconnects by itself, because it cannot tell
  "the app released the device" from "the device rebooted". When the app
  disconnects (its Disconnect button, or the tab closes) it posts a
  ``released`` notice on the same-origin ``BroadcastChannel("espp-device")``
  and the hub's banner says the device is free again.
- **Auto-reconnect.** Each console has an **auto-reconnect** checkbox (default
  on, remembered per origin in ``localStorage``). After an *unexpected* link
  loss — the device rebooted, was re-plugged, or crashed — the console retries
  on a bounded back-off (about 40 s, longer after a reboot it requested
  itself) and the moment the platform reports the device back, opening the
  same device by its ids. A manual Disconnect never triggers it. The System
  Console arms it on **Reboot** and disables it for a **Reboot into
  bootloader** (the ROM enumerates as a different USB device with no espp
  protocol); the OTA Console arms it after a finished update or a rollback so
  the post-reboot verify prompt appears by itself; the Core Dump Console arms
  it on the test-crash buttons.

All of it only works for pages served from a *secure context* — HTTPS (the
hosted apps), or plain HTTP on ``localhost`` (a local ``python -m
http.server`` in ``docs/apps``) — because WebUSB and Web Serial exist nowhere
else; a ``file://`` page is an opaque origin whose grants do not persist, so
it falls back to the chooser.
The shared helpers (``parseConnectParams``, the permitted device matchers
and the reconnect supervisor) are byte-identical in every console and the
hub; the hub's link builder (``connectQuery``) sits in its own block next to
them. Both are exercised by
``node components/dispatcher/web/test/resolve_module_id_test.js``.
