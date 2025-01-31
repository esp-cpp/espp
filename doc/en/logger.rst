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

Binary Logger
-------------

The binary logger provides a wrapper around
https://github.com/p-ranav/binary_log for enabling compact binary logging of
basic integer, floating point, and string data types.

Code examples for the binary logging API are provided in the `binary_logger`
example folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   logger_example
   binary_logger_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/logger.inc
.. include-build-file:: inc/binary-logger.inc
