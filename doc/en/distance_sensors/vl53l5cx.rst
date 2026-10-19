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

Web console
-----------

``espp::Vl53l5cxService`` (``vl53l5cx_service.hpp``) exposes a sensor over the
:doc:`dispatcher <../dispatcher/dispatcher>` / ``stream_frame`` protocol
(module id 10, protocol id ``espp.vl53l5cx``, see ``vl53l5cx_protocol.hpp``):
a host can configure every setting, start / stop ranging and receive each
ranging frame. The hosted `VL53L5CX console
<https://esp-cpp.github.io/espp/apps/vl53l5cx_console.html>`_ web app
(``components/vl53l/web/vl53l5cx_console.html``) draws the frames as a live 3D
surface or bar height field over WebUSB / Web Serial; the
:doc:`vl53l5cx_webapp_example` runs it next to the standard USB services.

.. ------------------------------- Examples ------------------------------------

.. toctree::

   vl53l5cx_example
   vl53l5cx_webapp_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/vl53l5cx.inc
.. include-build-file:: inc/vl53l5cx_protocol.inc
.. include-build-file:: inc/vl53l5cx_service.inc
