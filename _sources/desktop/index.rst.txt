Desktop APIs
************

.. toctree::
    :maxdepth: 1

    desktop

The `Desktop` component is a browser-rendered windowed desktop for
microcontrollers: the firmware describes **apps, windows and widgets** (a
retained tree with a small C++ app API) and the hosted desktop web app draws
and operates them over WebUSB / Web Serial — moving, resizing and closing
windows, pressing buttons, editing text, answering dialogs — and sends the
events back. It also provides ``ConsoleCapture``, a stdout / stderr tee into
a byte ring, which backs the example's Log Viewer app.
