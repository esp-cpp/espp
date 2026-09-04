Switch 2 Pro Controller
***********************

The `Switch2Pro` component emulates a **Nintendo Switch 2 Pro Controller over
BLE** so that a real Nintendo Switch 2 console accepts it as a native
controller — including pairing, waking the console from sleep, reconnecting, and
streaming input reports. It is built on :cpp:class:`espp::BleGattServer`
(NimBLE) and implements the reverse-engineered Nintendo custom GATT interface
(not HID-over-GATT) and the console's custom pairing handshake (not BLE SMP).

.. warning::

   **Supported targets: ESP32-C6** (and the other open-NimBLE-controller chips:
   C61/C2/H2) **and ESP32-S3** — pairing, input streaming, reconnect, and
   wake-from-sleep all verified against a real console. The S3 needs the console's
   sub-spec 5 ms interval, which is now official in ESP-IDF via
   ``CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE`` (default on) — use **ESP-IDF
   ≥ v6.1** (or the v6.0/v5.5/v5.4/v5.3 backports), where the S3 works with no
   binary patch. See the component README's "The 5 ms connection interval" and
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

The console drives the link at a 5 ms connection interval, below the 7.5 ms
Bluetooth spec minimum. It reaches 5 ms in every mode: a bonded *reconnect* or
*wake* connects at 5 ms from the ``CONNECT_IND``, and a *fresh* session is
renegotiated down to 5 ms (``LL_CONNECTION_UPDATE``) about 1.5 s after the
console subscribes to input. Only the initial pairing handshake (~15 ms, the
first ~1.5 s) works without sub-spec support; **sustained input, reconnect, and
wake all need it**. How to enable it depends on the chip:

- **ESP32-S3 / C3:** use **ESP-IDF ≥ v6.1** (or the v6.0/v5.5/v5.4/v5.3
  backports), where the official ``CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE``
  (default on; espressif/esp-idf#18467) makes the BTDM controller accept the 5 ms
  interval with **no binary patch**. This is the verified S3/C3 path.
- **ESP32-C6 / C61 / C2 / H2:** the open NimBLE controller has no such option, so
  enable the opt-in Kconfig option ``SWITCH2_PRO_PATCH_NIMBLE_5MS``, which
  binary-patches the prebuilt controller library in your global ``$IDF_PATH``
  install (off by default — it mutates your ESP-IDF install).

See the component README and ``tools/patch_nimble_5ms.py``.

.. ------------------------------- Example -------------------------------------

.. toctree::

   switch2_pro_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/switch2_pro.inc
.. include-build-file:: inc/switch2_pro_report.inc
.. include-build-file:: inc/switch2_pro_protocol.inc
