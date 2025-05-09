# Tabulate Component

[![Badge](https://components.espressif.com/components/espp/tabulate/badge.svg)](https://components.espressif.com/components/espp/tabulate)

The `tabulate` component provides a nice and easy way to perform pretty-printing
of tabular data to the console or file.

The `tabulate.hpp` header provides a convenience include / wrapper around
[p-ranav/tabulate](https://github.com/p-pranav/tabulate). It also exposes
`tabulate`'s include folder, so including `tabulate.hpp` is completely
equivalent to including both `tabulate/table.hpp`. Please see the documentation
for tabulate if you have any questions about usage beyond the examples provided
here.

## Example

The [example](./example) shows the use of the `tabulate` component to easily
create and pretty print tables to the console.
