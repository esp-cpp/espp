XIAO ESP32S3 Sense
******************

XIAO ESP32S3 Sense
------------------

The Seeed Studio XIAO ESP32S3 Sense combines the XIAO ESP32S3 module with the
Sense expansion board, which adds an OV2640 camera, a PDM microphone, and a
microSD card slot.

The `espp::XiaoEsp32S3Sense` component provides a singleton hardware abstraction
for the board's documented camera, microphone, user-LED, and microSD pin
mappings, as well as helpers for building the default ESP-IDF PDM RX microphone
configuration, mounting the onboard microSD card over SDSPI, and controlling the
onboard LED with a Gaussian breathing effect.

.. ------------------------------- Example -------------------------------------

.. toctree::

   xiao_esp32s3_sense_example.md

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/xiao-esp32s3-sense.inc
