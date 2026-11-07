Persistent Data APIs
********************

Persistent Data
---------------

The `PersistentData` class stores one data structure (a settings struct,
calibration data, a paired-device list) in a file on any filesystem reachable
through the VFS, tracks the schema version of what is on disk in a small
header (magic, version, a portable layout fingerprint, length, CRC32) and migrates older files forward
through a chain of `migrate()` steps when the structure changes. Saves are
atomic (temporary file plus rename), every operation reports a
`std::error_code` (`PersistentDataErrc`), and `encode()` / `decode()` expose
the same bytes for other storage backends.

Code examples for the Persistent Data API are provided in the `persistent_data`
example folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   persistent_data_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/persistent_data.inc
.. include-build-file:: inc/persistent_data_errc.inc
