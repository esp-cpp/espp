VL53L5CX Multizone ToF Sensor
*****************************

The `VL53L5CX
<https://www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html>`_ is a
multizone Time-of-Flight (ToF) sensor: it ranges up to 4 meters in every zone
of a 4x4 or 8x8 grid over a 63 degree diagonal field of view, at up to 60 Hz
(4x4) or 15 Hz (8x8). Every zone reports a distance, signal and ambient rates,
a sigma, a reflectance estimate and a validity status, and the sensor can run a
motion indicator over the zones.

The sensor has no firmware in ROM: the driver uploads the firmware (bundled,
from STMicroelectronics' Ultra Lite Driver, BSD-3-Clause) over I2C at every
initialization, which takes about two seconds at 400 kHz. The driver is a port
of ST's Ultra Lite Driver 1.1.2 to the espp peripheral conventions; one target
per zone is supported, and the crosstalk calibration / detection threshold
plugins are not ported yet.

.. note::

   The VL53L5CX shares this component with the single-zone
   :doc:`VL53L4CD / VL53L4CX driver <vl53l>` (both use 16-bit register
   addresses), but the two drivers share nothing else.

.. ------------------------------- Example -------------------------------------

.. toctree::

   vl53l5cx_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/vl53l5cx.inc
