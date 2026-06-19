CDR (Common Data Representation)
********************************

The ``cdr`` component provides a small, standalone Common Data Representation
reader/writer utility aimed at standards-oriented protocols such as DDS/RTPS.

This initial slice focuses on the pieces needed to start building interoperable
payloads without forcing applications to adopt DDS or RTPS as a whole:

- CDR / PL_CDR encapsulation identifiers
- endian-aware primitive serialization helpers
- CDR alignment and padding handling
- string helpers using the standard CDR length-prefix + null terminator layout
- body helpers for CDR fields embedded inside larger protocol elements
- fixed-array helpers and zero-copy payload/span views
- primitive sequence helpers
- standalone usage outside RTPS

Current scope:

- useful as a reusable building block for future DDS/RTPS payload work
- suitable for direct use in other protocols that want CDR-style payloads
- not yet a full XTypes / XCDR2 implementation

.. toctree::

   cdr_example

API Reference
-------------

.. include-build-file:: inc/cdr.inc
