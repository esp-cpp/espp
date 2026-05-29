I2C
***

The `i2c` component provides C++ wrappers built on ESP-IDF's I2C drivers.

There are two driver families available:

- the deprecated legacy driver
- the new master/slave bus API used by ESP-IDF v6.x

Only one family can be selected at a time via Kconfig.

Primary API
===========

When the new driver family is selected, ``espp::I2c`` remains the main public
API. It preserves the familiar address-based helper methods for backwards
compatibility while internally using ESP-IDF's new master bus/device model.

That means existing code using methods such as ``probe_device()``, ``read()``,
``write()``, ``write_read()``, and ``read_at_register()`` can keep working on
ESP-IDF v6.x without being rewritten around the raw ESP-IDF handles.

For code that wants explicit per-device ownership, ``espp::I2c`` also exposes
``add_device()`` which returns an ``I2cMasterDevice``.

``I2cMenu`` is available for the bus-compatible ``espp::I2c`` API and can be
used to scan the bus, probe devices, and read/write registers interactively.

.. ------------------------------- Example -------------------------------------

.. toctree::

   i2c_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/i2c.inc
.. include-build-file:: inc/i2c_menu.inc

Explicit master/slave APIs
==========================

The component also exposes the explicit new-driver wrappers directly:

- ``I2cMasterBus``
- ``I2cMasterDevice``
- ``I2cSlaveDevice``

These classes mirror the ESP-IDF master/slave bus model more closely. The master
helpers are synchronous/blocking wrappers around ESP-IDF's new API.

``I2cMasterMenu`` and ``I2cMasterDeviceMenu`` are also provided for interactive
testing of explicit master buses and devices.

``I2cSlaveDevice`` and ``I2cSlaveMenu`` are available for slave-side work.
The slave wrapper follows ESP-IDF's callback-driven model: ``read()`` returns
the next complete master-write transaction buffered by the receive callback
path, ``write()`` stages bytes for the next master read, and optional request /
receive callbacks run in task context instead of the ISR callback itself.

If you need the exact transaction size, use the ``read()`` overload that
returns the received length by reference. The original boolean-only ``read()``
API remains available for compatibility.

On ESP-IDF v5.5's default slave driver, the receive callback does not expose the
exact byte count. ESPP still supports buffered reads and receive callbacks in
that configuration, but trailing unread bytes are zero-filled and the reported
receive length matches the requested read size.

There is not yet a dedicated slave example.

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

Legacy API
==========

The legacy ESP-IDF I2C driver is still available behind
``CONFIG_ESPP_I2C_USE_LEGACY_API`` for older environments, but it is deprecated
and should not be used for new development.
