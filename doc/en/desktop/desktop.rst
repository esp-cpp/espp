Desktop, Desktop Service & Console Capture
******************************************

The `Desktop` class is the retained model of a windowed desktop: **apps**
(name, icon, description and a launch callback), **windows** (title, flags,
geometry) each holding a tree of **widgets** (containers — Column, Row, Group
— and leaves — Label, Button, Checkbox, TextBox, TextArea, List, Table,
Select, Progress, Slider, Separator, Spacer), plus modal **dialogs** (message
and input boxes) and **notifications**. The firmware never draws anything: a
browser connected through the `DesktopService` renders the desktop, lets the
user operate it, and reports every interaction back as an event.

An app is a few lines of C++: register it, and build its window in the launch
callback with the ``Window`` / ``Widget`` value handles (``win.label(...)``,
``win.button("+1", [=]{ ... })``, ``label.set_text("Count: {}", n)``). Every
mutator may be called from any task; it records the change and wakes the
desktop task, which coalesces everything changed since the last flush (last
value wins, text appends concatenate, a removed widget cancels its pending
changes) into the fewest frames every ``Config::flush_period`` (50 ms by
default). Every application callback — launch, widget / window / dialog
handlers, timers, ``post()`` — runs on the desktop task with no lock held, so a
handler may call anything, including closing its own window. Handles are
plain values that become invalid (and no-ops) once their target is gone.

Layout is a box model: nested Column / Row / Group containers become CSS
flexbox in the browser; a widget's ``weight`` is its flex-grow along the
parent's axis and its ``layout`` bits set the cross-axis behaviour (stretch,
scroll, align end / center); ``width`` / ``height`` give a preferred size.
Windows carry flags (movable, resizable, closable, modal, minimizable,
maximizable, centered, pinned, wants-geometry) and a geometry the browser may
override with what the user last chose (remembered per app and title).

The `DesktopService` class serves one ``Desktop`` over **any byte stream** as
a :doc:`dispatcher <../dispatcher/dispatcher>` module (``espp.desktop`` v1,
module id 9 by default; one instance per transport). It only decodes and
validates: a malformed request is answered with ``ERROR``, everything else is
handed to the desktop and handled — replied to, and its events broadcast — on
the desktop task. ``GET_DESKTOP`` returns the app list and settings followed
by the full tree of every open window and dialog (so a reconnecting browser
resyncs in one request) and marks the transport active; ``LAUNCH_APP`` /
``CLOSE_WINDOW`` are acknowledged; ``WINDOW_EVENT`` / ``WIDGET_EVENT`` /
``DIALOG_RESULT`` are not. Payloads are split across frames (a long text
becomes Text + TextAppend pieces, a long list several ranges, a big window
tree WINDOW_OPEN + WIDGET_ADD continuations), never truncated; the wire format
is documented in ``include/detail/desktop_protocol.hpp`` and checked by a
host test against the fixture ``test/desktop_vectors.txt``, which the web
app's test reads too.

The `ConsoleCapture` class keeps the last N bytes of everything the firmware
prints (``ESP_LOG``, ``espp::Logger``, ``printf``, stderr) in a byte ring a
reader can page through with a cursor, while still writing them to the
original console: a tiny write-only VFS device is registered and stdout /
stderr are re-opened on it. It is what the example's Log Viewer app streams.
It is mutually exclusive with ``UsbDevice::route_console_to_cdc()`` (both
re-point stdout).

The hosted `espp Desktop <https://esp-cpp.github.io/espp/apps/desktop.html>`_
web app (``web/desktop.html``) speaks the protocol over **WebUSB** or **Web
Serial**: a desktop with app icons and a start menu, draggable / resizable
windows with a taskbar, modal dialogs and toasts, and a frame log for
debugging.

.. ------------------------------- Example -------------------------------------

.. toctree::

   desktop_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/desktop.inc
.. include-build-file:: inc/desktop_service.inc
.. include-build-file:: inc/console_capture.inc
