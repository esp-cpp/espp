Remote Debug APIs
*****************

The `RemoteDebug` class provides a web-based interface for remote debugging and monitoring of ESP32 devices. It offers:

* **GPIO Control**: Configure pins as input/output and control their states remotely
* **ADC Monitoring**: Real-time voltage monitoring with live graphs and configurable sampling rates
* **Console Log Viewing**: Capture and display stdout output with ANSI color support
* **Multi-Client Support**: Efficient batched updates to support multiple simultaneous web clients

The component uses the espp WiFi, ADC, and FileSystem components for functionality, and provides a responsive web interface accessible from any browser on the local network.

.. ------------------------------- Example -------------------------------------

.. toctree::

   remote_debug_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/remote_debug.inc
