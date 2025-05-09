# Comma-Separated Value (CSV) Parsing Component

[![Badge](https://components.espressif.com/components/espp/csv/badge.svg)](https://components.espressif.com/components/espp/csv)

The `csv.hpp` header provides a convenience include / wrapper around
`p-ranav/csv2 <https://github.com/p-pranav/csv2`_. It also exposes `csv2`'s
include folder, so including `csv.hpp` is completely equivalent to including
both `csv2/reader.hpp` and `csv2/writer.hpp`. Please see the documentation for
csv2 if you have any questions about usage beyond the examples provided here.

## Example

The [example](./example) shows the use of the `csv` component to read and write
CSV data.
