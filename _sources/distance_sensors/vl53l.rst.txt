VL53LXX ToF Distance Sensor
***************************

The `VL53L4CD
<https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cd.html>`_ and
`VL53L4CX
<https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cx.html>`_ are
Time-of-Flight (ToF) distance sensors. They can measure distance from a target
object without touching it. The sensors use a laser to send a pulse of light
and measure the time it takes for the light to reflect back. The VL53L4CD can
measure absolute distances up to 1.3 meters, while the VL53L4CX can reach up
to 6 meters, both with 1mm resolution.

.. note::

   This component supports the **VL53L4CD** and **VL53L4CX** sensors, which
   use a 16-bit register addressing scheme. It is **not** compatible with the
   older VL53L0X or VL53L1X sensors, which use a different register map.

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
