# CDR Example

This example demonstrates a small CDR round-trip using the `cdr` component.

It exercises:

- a little-endian CDR encapsulation header
- primitive value serialization
- CDR string serialization
- `uint16_t` sequence serialization
- fixed-array serialization with `write_array` / `read_array`
- headerless/body CDR helpers for embedding fields inside a larger protocol value
- round-trip parsing with `CdrReader`

## How to use example

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```bash
idf.py -p PORT flash monitor
```

Replace `PORT` with the name of the serial port to use.

## Expected Output

The example logs the encoded byte count and the decoded values. It finishes by
printing `CDR round-trip succeeded`.
