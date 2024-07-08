I2C
***

The `I2C` class provides a simple interface to the I2C bus. It is a wrapper
around the esp-idf I2C driver.

A helper `I2cMenu` is also provided which can be used to interactively test
I2C buses - scanning the bus, probing devices, reading and writing to devices.

.. ------------------------------- Example -------------------------------------

.. toctree::

   i2c_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/i2c.inc
.. include-build-file:: inc/i2c_menu.inc
