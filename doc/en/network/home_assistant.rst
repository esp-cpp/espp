Home Assistant
**************

The ``HomeAssistant`` component provides pragmatic Home Assistant integration
for ESP-IDF devices.

Supported transports and helpers:

- MQTT discovery and state / command topics
- REST API helpers returning raw JSON responses
- WebSocket API connect / auth / subscribe helpers with raw JSON callbacks

The first release intentionally supports a practical subset of discovery fields
for these entity types:

- ``sensor``
- ``binary_sensor``
- ``button``
- ``switch``
- ``number``
- ``text``
- ``fan``
- ``cover``
- ``climate``

.. ------------------------------- Example -------------------------------------

.. toctree::

   home_assistant_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/home_assistant.inc
