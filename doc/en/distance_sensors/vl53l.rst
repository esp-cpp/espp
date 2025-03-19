VL53LXX ToF Distance Sensor
***************************

The `Vl53L0X
<https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html>`_ is a
Time-of-Flight (ToF) distance sensor. It is a sensor that can measure distance
from a target object without touching it. The sensor uses a laser to send a
pulse of light and measures the time it takes for the light to reflect back to
the sensor. The sensor can measure distance up to 2 meters with a resolution of
1mm. Because of the way the sensor works, it is able to measure multiple
distances at the same time.

The sensor can struggle with outdoor use, as sunlight can interfere with the
sensor's ability to measure distance. The sensor is also not able to measure
distance through glass or other transparent materials.

.. ------------------------------- Example -------------------------------------

.. toctree::

   vl53l_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/vl53l.inc
