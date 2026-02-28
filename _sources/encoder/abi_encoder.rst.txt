ABI Encoder
***********

The `AbiEncoder` allow the user a configurable container wrapping the `pulse
count
<https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/pcnt.html>`_
ESP peripheral api as it would be configured for an incremental encoder with
quadrature output (see `Wikipedia
<https://en.wikipedia.org/wiki/Incremental_encoder>`_). The AbiEncoder can be
configured to be either `LINEAR` or `ROTATIONAL`, and provides access to the
current `count` of the encoder (including the overflow underflow conditions).

Code examples for the ABI Encoder are provided in the `encoder` example folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   encoder_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/abi_encoder.inc
