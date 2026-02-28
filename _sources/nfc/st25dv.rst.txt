ST25DV
******

The `St25dv` provides a utility class for NFC communications using the ST25DV
Dynamic NFC / RFID tag. The tag includes 4Kb, 16Kb, or 64Kb EEPROM along with
support for fast transfer mode - which enables bi-directional communications
with other NFC/RFID enabled devices such as phones. This chip stores in its
EEPROM NFC T5T tag records - which include a CC header (that this class
maintains) followed by serialized NDEF records.

.. ------------------------------- Example -------------------------------------

.. toctree::

   st25dv_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/st25dv.inc
