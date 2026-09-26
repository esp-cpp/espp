LSM6DSO 6-Axis IMU
******************

The `Lsm6dso` component provides a driver for the LSM6DSO 6-Axis Inertial
Measurement Unit (IMU) from STMicroelectronics and compatible LSM6DS-family
parts that share the same basic accel / gyro register map.

For basic accel / gyro operation, the driver also accepts the LSM6DS33
``WHO_AM_I`` value (`0x69`), allowing boards such as MotorGo Plink to reuse the
same component over I2C without a separate IMU driver.

.. ------------------------------- Example -------------------------------------

.. toctree::

   lsm6dso_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/lsm6dso.inc
.. include-build-file:: inc/lsm6dso_detail.inc
