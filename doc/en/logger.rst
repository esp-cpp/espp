Logging APIs
************

Format
------

Format is a simple component which exposes `libfmt` for use within esp-idf as a
single include.

Logger
------

The logger provides a cross-platform wrapper around `libfmt` for providing
configurable log output with different levels that can be turned on / off at
runtime.

Code examples for the logging API are provided in the `logger` example folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   logger_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/logger.inc
