# CDR (Common Data Representation)

Reflection-driven CDR/XCDR serialization for plain C++ structs — no IDL
compiler, no hand-written read/write call sequences. The compiler generates
the serialization code from the struct definition itself (the same usability
pattern as the `serialization` component's alpaca, with a DDS/RTPS-compatible
wire format instead).

The library is [finger563/cdr](https://github.com/finger563/cdr), vendored as
a git submodule under `detail/`; this component wires it into ESP-IDF and
depends on the `reflect_cpp` component for its reflection backend. See the
library's `README.md` for the supported type mapping and
`docs/DESIGN.md` for the architecture and wire-format rules.

```cpp
struct ImuSample {
  uint64_t stamp_us;
  std::array<float, 3> accel;
  std::array<float, 3> gyro;
  float temperature;
};

auto bytes = cdr::serialize(sample);              // XCDR2, appendable (default)
auto ros2  = cdr::serialize<cdr::xcdr1>(sample);  // ROS 2 / classic-CDR peers
auto back  = cdr::deserialize<ImuSample>(*bytes); // std::expected<ImuSample, cdr::error>
```

Highlights:

- XCDR1 (plain CDR — ROS 2, CycloneDDS defaults) and XCDR2 (plain +
  delimited/appendable with DHEADER — FastDDS, OpenDDS defaults), both
  endiannesses, with appendable schema evolution in both directions.
- Wire format byte-verified against pycdr2 (CycloneDDS's codec); the Python
  side of a message is a plain `pycdr2` dataclass.
- `std::expected` error handling with error code, payload offset, and field
  name; bounds-checked, fuzz-tested deserializers.
- `cdr::param_list_writer` / `param_list_reader` for the PL_CDR parameter
  lists RTPS discovery (SPDP/SEDP) uses.
- Zero-allocation `cdr::serialize_into` and body-only variants for RTPS
  submessage composition.

Requires C++23 (`std::expected`), the default on ESP-IDF 5.2+ toolchains.

This component replaces the previous manual `espp::CdrWriter`/`espp::CdrReader`
API (an imperative XCDR1-only reader/writer); the RTPS component and examples
have been migrated to the reflection-driven API.

## Example

The [example](./example) shows struct round-trips in both XCDR versions, the
zero-allocation path, error handling, and PL_CDR parameter lists.
