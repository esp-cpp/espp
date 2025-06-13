I2C
***

The `i2c` component provides c++ wrappers / implementation build on ESP-IDF's
i2c drivers.

There are two versions of these drivers, supporting the different driver
versions supported in ESP-IDF v5.x.

The two drivers cannot coexist at the same time, so you must select (via
kconfig) which driver you want to use.

Legacy API
==========

This driver is deprecated and will no longer be available for ESP-IDF >= v5.5 -
so make sure you upgrade.

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

New API
=======

This driver was introduced and has been refined in ESP-IDF v5. As of v5.5, it
will be the only driver available.

The `I2cMasterBus` and `I2cMasterDevice` classes provide simple interfaces to
manage and communicate with I2C peripherals on an I2C bus.

It should be noted that while the new ESP-IDF APIs can support asynchronous
mode, these classes do not.

Helper `I2cMasterMenu` and `I2cMasterDeviceMenu` are also provided which can be
used to interactively test I2C buses - scanning the bus, probing devices, and
performing read/write operations with devices.

There are also `I2cSlaveDevice` and `I2cSlaveMenu` classes for developing your
own I2C Slave device using ESP, though they are not currently tested / used as
the upstream ESP-IDF is only recently stabilizing.

.. ------------------------------- Example -------------------------------------

.. toctree::

   i2c_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/i2c_master.inc
.. include-build-file:: inc/i2c_master_menu.inc
.. include-build-file:: inc/i2c_master_device_menu.inc
.. include-build-file:: inc/i2c_slave.inc
.. include-build-file:: inc/i2c_slave_menu.inc
