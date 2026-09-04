Switch 2 Pro Controller
***********************

The `Switch2Pro` component emulates a **Nintendo Switch 2 Pro Controller over
BLE** so that a real Nintendo Switch 2 console accepts it as a native
controller — including pairing, waking the console from sleep, reconnecting, and
streaming input reports. It is built on :cpp:class:`espp::BleGattServer`
(NimBLE) and implements the reverse-engineered Nintendo custom GATT interface
(not HID-over-GATT) and the console's custom pairing handshake (not BLE SMP).

.. warning::

   **Supported target: ESP32-C6** (and the other open-NimBLE-controller chips:
   C61/C2/H2), where pairing, input streaming, reconnect, and wake-from-sleep
   all work against a real console. The **ESP32-S3** also builds and pairs; its
   5 ms reconnect/wake support is now official in ESP-IDF via
   ``CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE`` (default on, recent IDF
   required — see the README), but full on-hardware S3 verification is still
   pending. See the component README's "The 5 ms connection interval" and
   "Known issues".

.. note::

   Interoperability only. This component contains no Nintendo or Espressif
   binaries; the pairing "authentication" relies on a published fixed key and is
   a possession check, not per-device attestation.

.. code-block:: cpp

   #include "switch2_pro.hpp"

   espp::Switch2Pro controller({.device_name = "Pro Controller"});
   controller.init();  // verifies pairing crypto, builds GATT, advertises

   // feed input state; a driver-owned task streams it once the console subscribes
   espp::switch2::Pro2InputReport report;
   report.set_a(true);
   controller.set_input_report(report);

The 5 ms connection interval
----------------------------

A real controller *reconnects* at a 5 ms connection interval, below the 7.5 ms
Bluetooth spec minimum, chosen by the console in its ``CONNECT_IND``. Accepting
it (required for reconnect and wake-from-sleep) needs the opt-in Kconfig option
``SWITCH2_PRO_PATCH_NIMBLE_5MS``, which binary-patches the prebuilt BLE
controller library in your global ``$IDF_PATH`` install. It is **off by
default** (it mutates your ESP-IDF install); fresh pairing and first-session
input work without it. See the component README and ``tools/patch_nimble_5ms.py``.

.. ------------------------------- Example -------------------------------------

.. toctree::

   switch2_pro_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/switch2_pro.inc
.. include-build-file:: inc/switch2_pro_report.inc
.. include-build-file:: inc/switch2_pro_protocol.inc
