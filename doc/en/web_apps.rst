Web Apps
********

espp ships a growing set of **self-contained browser tools** that talk directly
to your hardware using the Web Serial / WebUSB / WebHID APIs (Chromium-based
browsers) — nothing to install. They are hosted alongside this documentation:

    **→** `esp-cpp.github.io/espp/apps <https://esp-cpp.github.io/espp/apps/index.html>`_

Highlights:

- **Board Console & ESP Flasher** — a general-purpose Web Serial monitor with
  reset / bootloader controls and an `esptool-js`-based firmware flasher.
- **ODrive ASCII Web Serial console** and **WebUSB console** — interactive
  terminals with quick motor controls and live plotting for the
  :doc:`odrive_ascii <motor_control/odrive_ascii>` protocol server.
- **ODrive Native WebUSB control panel** — endpoint-tree browser, live
  multi-signal plots, and typed read/write for the ODrive native (Fibre)
  binary protocol.
- **WebHID input visualizer** — decoded buttons / sticks / raw reports for any
  HID device, driven entirely by its report descriptor.

Adding a new app
----------------

Any single-file app placed in a component's ``web/`` directory
(``components/<name>/web/*.html``, plus optional same-origin ``.js`` assets) is
hosted automatically by the docs workflow, and the `apps landing page
<https://esp-cpp.github.io/espp/apps/index.html>`_ lists it using the file's
``<title>`` and ``<meta name="description">`` tags — no registry to maintain.
Apps must be fully self-contained (no CDN resources) so they work offline and
under GitHub Pages' strict hosting.
