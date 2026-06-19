# CDR (Common Data Representation) Component

[![Badge](https://components.espressif.com/components/espp/cdr/badge.svg)](https://components.espressif.com/components/espp/cdr)

The `cdr` component provides a small, standalone Common Data Representation
(CDR) reader/writer utility aimed at standards-oriented protocols such as
DDS/RTPS.

This initial slice is intentionally focused on the most immediately useful
pieces for building interoperable payloads:

- encapsulation identifiers for `CDR_BE`, `CDR_LE`, `PL_CDR_BE`, and `PL_CDR_LE`
- endian-aware primitive read/write helpers
- CDR alignment and padding handling
- string serialization helpers using the standard CDR length-prefix + null terminator format
- headerless/body helpers for CDR fields embedded inside larger protocol elements
- fixed-array helpers and zero-copy payload/span views
- sequence helpers for homogeneous primitive collections
- standalone usage without depending on RTPS or DDS layers

Current scope:

- good fit for building RTPS payloads and parameter lists incrementally
- designed to stay reusable outside DDS/RTPS
- **not** yet a full DDS XTypes / XCDR2 implementation

## Example

The [example](./example) demonstrates a small round-trip using:

- a little-endian CDR encapsulation header
- primitive values
- a CDR string
- a `uint16_t` sequence
