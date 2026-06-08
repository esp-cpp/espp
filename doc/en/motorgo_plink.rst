MotorGo Plink
*************

MotorGo Plink
-------------

The MotorGo Plink is a four-channel ESP32-S3 motor-control board from Every
Flavor Robotics. The `espp::MotorGoPlink` component provides a singleton board
abstraction for the documented pin mapping of its DC motor outputs, encoder
chip-selects, servo signal header, onboard LSM6DS33 IMU, Qwiic and internal I2C
buses, and user/status LEDs.

It also provides helper methods for:

- initializing the four dual-PWM motor channels and commanding normalized motor
  speeds, where positive commands drive ``pwm_a`` and negative commands drive
  ``pwm_b``
- initializing the shared SSI/SPI encoder bus and creating four
  `espp::Mt6701<SSI>` helpers
- initializing the onboard LSM6DS33 IMU on the hidden I2C bus via the
  `espp::Lsm6dso` helper
- accessing the four RC-servo signal pins for use with an external servo driver
- controlling the user and status LEDs with direct brightness setters or a
  Gaussian breathing effect

.. ------------------------------- Example -------------------------------------

.. toctree::

   motorgo_plink_example.md

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/motorgo-plink.inc
