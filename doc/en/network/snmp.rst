SNMP
****

The `Snmp` component provides a synchronous SNMP manager for ESP-IDF.

It supports:

- SNMPv2c
- SNMPv3 USM with the practical first-release `authPriv` profile
- BER/ASN.1 handling for common SNMP scalar and application data types
- generic GET, GETNEXT, GETBULK, SET, and WALK operations
- convenience helpers for the SNMPv2-MIB system group and common IF-MIB columns

The current SNMPv3 implementation targets SHA-1 authentication and AES-128
privacy. Discovery, engine-ID caching, and timeliness/report handling are
implemented to support interoperability with typical network devices.

.. ------------------------------- Example -------------------------------------

.. toctree::

   snmp_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/snmp.inc
