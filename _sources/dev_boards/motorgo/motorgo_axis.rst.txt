MotorGo Axis
************

MotorGo Axis
------------

The MotorGo Axis is a two-channel ESP32-S3 BLDC motor-control board from Every
Flavor Robotics. The ``espp::MotorGoAxis`` component provides a singleton board
abstraction for the documented pin mapping of its 6-PWM motor outputs, encoder
chip-selects, onboard LSM6DS33 IMU, Qwiic and internal I2C buses, and
user/status LEDs.

It also provides helper methods for:

- initializing the two 6-PWM motor channels via ``espp::BldcDriver``
- initializing the shared SSI/SPI encoder bus and creating two
  ``espp::Mt6701<SSI>`` helpers
- initializing the onboard LSM6DS33 IMU on the hidden I2C bus via the
  ``espp::Lsm6dso`` helper
- controlling the user and status LEDs with direct brightness setters or a
  Gaussian breathing effect

.. ------------------------------- Example -------------------------------------

.. toctree::

   motorgo_axis_example.md

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/motorgo-axis.inc
