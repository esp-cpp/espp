Command Line Interface (CLI) APIs
*********************************

The `cli.hpp` header provides a convenience include / wrapper around
`daniele77/cli <https://github.com/daniele77/cli>`_. It also exposes `cli`'s
include folder, so including `"cli.hpp"` is completely equivalent to including
`<cli/cli.h>` - but you can additionally include other files from
`<cli/>`. It also 2 conveniences:

1. a `cli_configure_stdin_stdout()` function which sets up the esp UART to
   enable blocking `std::cin` read, which is needed for `std::cin` to work (and
   therefore - currently - for `cli` to work), and
2. a `espp::Cli` class which inherits from `cli::CliSession` and has a
   near-identical implementation as `cli`'s `CliFileSesson`, with the exception
   that it explicitly reads character by character intead of line by line and
   prints out the characters as they are being input.

Please see the documentation for `cli` if you have any questions about
usage beyond the examples provided here.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/cli.inc
