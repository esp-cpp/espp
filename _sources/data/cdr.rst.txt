CDR (Common Data Representation)
********************************

The ``cdr`` component provides reflection-driven CDR/XCDR serialization for
plain C++ structs — the compiler generates the serialization code from the
struct definition itself. There is no IDL compiler and no hand-written
read/write call sequence; defining an aggregate struct is all it takes:

.. code-block:: cpp

   struct ImuSample {
     uint64_t stamp_us;
     std::array<float, 3> accel;
     std::array<float, 3> gyro;
     float temperature;
   };

   auto bytes = cdr::serialize(sample);              // XCDR2, appendable (default)
   auto ros2  = cdr::serialize<cdr::xcdr1>(sample);  // ROS 2 / classic-CDR peers
   auto back  = cdr::deserialize<ImuSample>(*bytes); // std::expected<ImuSample, cdr::error>

The implementation is the `finger563/cdr <https://github.com/finger563/cdr>`_
library, vendored as a git submodule under ``detail/``, with the
:doc:`reflect_cpp <reflect_cpp>` component as its reflection backend. See the
library's `README <https://github.com/finger563/cdr#readme>`_ for the
supported type mapping and its
`design document <https://github.com/finger563/cdr/blob/main/docs/DESIGN.md>`_
for the architecture and wire-format rules.

Feature highlights:

- XCDR1 (plain CDR — what ROS 2 and CycloneDDS speak by default) and XCDR2
  (plain + delimited/appendable with DHEADER — what Fast DDS and OpenDDS
  speak by default), in both endiannesses
- appendable schema evolution in both directions: newer peers' extra members
  are skipped, missing members keep their defaults
- wire format byte-verified against ``pycdr2`` (CycloneDDS's codec); the
  Python side of a message is a plain ``pycdr2`` dataclass — no bindings
- ``std::expected`` error handling carrying an error code, payload offset,
  and the field name that failed; bounds-checked, fuzz-tested deserializers
- ``cdr::bounded_string<N>`` / ``cdr::bounded_vector<T, N>`` for IDL bounded
  types
- ``cdr::param_list_writer`` / ``cdr::param_list_reader`` for the PL_CDR
  parameter lists used by RTPS discovery (SPDP/SEDP)
- zero-allocation ``cdr::serialize_into`` and body-only variants for
  composing RTPS submessages

Everything lives in the ``cdr`` namespace (not ``espp``), matching the
standalone library. Requires C++23 (``std::expected``), the default C++
standard on ESP-IDF 5.2+ toolchains.

.. note::

   This component replaces the earlier manual ``espp::CdrWriter`` /
   ``espp::CdrReader`` API. The ``rtps`` component and examples have been
   migrated; payloads previously built with hand-written write/read call
   sequences are now plain structs.

.. toctree::

   cdr_example

API Reference
-------------

.. include-build-file:: inc/cdr.inc
