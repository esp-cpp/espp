NVS APIs
********

NVS
---

The `NVS` component provides a simple class representing an NVS controller. 

NVSHandle
---------

The `NVSHandle` class manages individual NVS storage handles, allowing for scoped
control over specific NVS namespaces. It simplifies operations like reading,
writing, and committing key-value pairs within these namespaces. 

Code examples for the NVS and NVSHandle API are provided in the `nvs` example folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   nvs_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/nvs.inc
.. include-build-file:: inc/nvs_handle_espp.inc
