Base Peripheral
***************

The `espp::BasePeripheral` class is a base class for all peripherals. It
provides a common interface for all peripherals and is used to access the
peripheral's registers. It is primarily designed to be used as a base class for
peripheral classes that communicate using I2C (address-based) or SPI (CS-based)
protocols.

The base class provides an interface for specifying different communications
functions that the peripheral may use, as well as providing some base
implementations for common functionality such as reading / writing u8 and u16
values from / to a register.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/base_peripheral.inc
