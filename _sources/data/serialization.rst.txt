Serialization APIs
******************

.. _alpaca-section:

Alpaca
------

The serialization library is a light wrapper around the third-party `alpaca
<https://github.com/p-ranav/alpaca>`_ serialization library.

Serialization
-------------

The `Serialization` component provides a simple wrapper with reasonable default
options around the :ref:`alpaca-section` libarary. The default
serialization/deserialization options are configured such that messages / types
can be distinguished from each other.

Code examples for the serialization API are provided in the `serialization`
example folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   serialization_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/serialization.inc
