# CDR Example

This example demonstrates the reflection-driven `cdr` component: the compiler
generates CDR serialization code directly from a plain struct definition.

It exercises:

- struct serialization to XCDR2 (appendable, the FastDDS/OpenDDS default) and
  XCDR1 (plain CDR, the ROS 2 / CycloneDDS default)
- deserialization driven by the received encapsulation header (version and
  endianness)
- `std::expected` error handling with error code, payload offset, and field
  name
- the zero-allocation `cdr::serialize_into` path and `cdr::serialized_size`
- PL_CDR parameter-list writing and reading (the encoding RTPS SPDP/SEDP
  discovery uses)

## How to use example

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```bash
idf.py -p PORT flash monitor
```

Replace `PORT` with the name of the serial port to use.

## Expected Output

The example logs the serialized sizes for both XCDR versions, the round-tripped
field values, the zero-allocation write size, and the parameters found in the
PL_CDR parameter list, finishing with `example complete`.
