Telemetry
*********

The ``telemetry`` component pairs a self-contained browser **data plotter**
web app with an optional firmware-side **binary telemetry** emitter, so you can
plot live data from a device two complementary ways:

- **Text / CSV over Web Serial** — point the web app at any device that prints
  columnar data (a header line plus numeric rows). It auto-detects the delimiter
  and schema, discards non-matching lines (including ESP-IDF log lines), and
  re-evaluates when a new header arrives. No firmware component is required.
- **Binary telemetry over WebUSB** — the `Telemetry` class streams typed float
  channels directly, for higher rate, lower overhead, and device-accurate
  timestamps. The same web app decodes and plots them.

The hosted app is at
`esp-cpp.github.io/espp/apps/telemetry.html
<https://esp-cpp.github.io/espp/apps/telemetry.html>`_ (Chromium, secure
context). It plots a high number of points efficiently (``uPlot``) with drag
zoom, a per-series filter, and CSV save / load, plus optional 2D X–Y and 3D
X–Y–Z modes over the same parsed columns.

Telemetry service
-----------------

The `Telemetry` class is a small device→host protocol carried on the espp
:doc:`stream_frame <../stream_frame/stream_frame>` framing (dispatcher module
id 3), so it can share one USB vendor / CDC stream with other modules via
:doc:`dispatcher <../dispatcher/dispatcher>`. Firmware declares a fixed set of
named ``float`` channels (the **SCHEMA**) and pushes **SAMPLE** frames — a
device timestamp plus one float per channel (batchable) — with ``emit(...)``.
Host requests are ``GET_SCHEMA`` and ``SET_STREAM`` (enable/disable + rate).

Construct it with the channel names and a ``send`` function, register
``handle()`` on a :doc:`dispatcher <../dispatcher/dispatcher>` module (or feed
raw bytes to ``feed()``), and call ``emit()`` from your producer. Frames are
built under an internal mutex and the ``send`` callback runs with the lock
released, so ``emit()`` and request handling are safe to call concurrently. See
the example for USB vendor (WebUSB) wiring and capability discovery that lists
the app in the browser Device Hub. (The framing is transport-agnostic — CDC /
UART / a socket work too — but the web app's binary path consumes WebUSB.)

.. ------------------------------- Example -------------------------------------

.. toctree::

   telemetry_example

API Reference
-------------

.. include-build-file:: inc/telemetry.inc
