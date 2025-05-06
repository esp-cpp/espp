# Binary Logging Component

The binary log component provides a wrapper around
https://github.com/p-ranav/binary_log for enabling compact binary logging of
basic integer, floating point, and string data types.

This component simply exposes the
[p-ranav/binary_log](https://github.com/p-ranav/binary_log) binary logger, with
some modifications in a fork
[finger563/binary_log](https://github.com/finger563/binary_log) to allow packer
customization as is required on embedded systems such as with ESP-IDF.

The `binary-log` does not support all datatypes, but only some base datatypes
such as integer types, float types, and string types.

The logger produces the following data:
1. Log data - these are indexes and the arguments which point to either the
   index data or the runfile data.
2. Index data - these are static data (format strings, constant arguments, etc.)
3. Runfile data - these are indices of the log file which run multiple times in
   a row.
   
For more information about the logging, its format, and the types supported,
please see [p-ranav/binary_log](https://github.com/p-ranav/binary_log).

## Example

The [example](./example) shows how to use the `binary-log` component to format
and store logs in a binary packed form, either to disk or into a ringbuffer in
memory.

