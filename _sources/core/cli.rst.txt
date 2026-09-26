Command Line Interface (CLI) APIs
*********************************

The `cli.hpp` header provides a convenience include / wrapper around
`daniele77/cli <https://github.com/daniele77/cli>`_. It also exposes `cli`'s
include folder, so including `"cli.hpp"` is completely equivalent to including
`<cli/cli.h>` - but you can additionally include other files from
`<cli/>`. It also provides a few conveniences:

1. a `espp::Cli` class which inherits from `cli::CliSession` and has a
   near-identical implementation as `cli`'s `CliFileSesson`, with the exception
   that it explicitly reads character by character instead of line by line and
   prints out the characters as they are being input, and
2. a static `espp::Cli::configure_stdin_stdout()` function which sets up the esp
   UART to enable blocking `std::cin` read, which is needed for `std::cin` to
   work (and therefore - currently - for `cli` to work). This function is
   automatically called by the `espp::Cli` constructor, but it can also be
   called manually without needing to create a `espp::Cli` object if you wish to
   simply use the `std::cin` and related functions.
3. a `espp::LineInput` class which provides support for an interactive input
   system which can optionally have history with navigation. It allows for
   getting the user input as an arbitrary input string while enabling some
   keyboard movement and editing commands. The `espp::Cli` uses the
   `espp::LineInput` class internally to handle the input aspects of the Cli,
   but the `espp::LineInput` class can be re-used anywhere you want to get one
   or more lines of user input in a reusable way. Note that if you use it in
   another class / outside the `espp::Cli` class, you will need to call the
   `espp::Cli::configure_stdin_stdout()` as mentioned above.

Please see the documentation for the `cli submodule
<https://github.com/daniele77/cli>`_ if you have any questions about usage
beyond the examples provided here.

.. ------------------------------- Example -------------------------------------

.. toctree::

   cli_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/cli.inc
.. include-build-file:: inc/line_input.inc
