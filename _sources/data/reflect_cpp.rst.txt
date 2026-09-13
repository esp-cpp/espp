Reflect-C++ (compile-time reflection)
*************************************

The ``reflect_cpp`` component vendors the reflection core of the third-party
`reflect-cpp <https://github.com/getml/reflect-cpp>`_ library (as a git
submodule under ``detail/``), giving espp components compile-time reflection
over plain aggregate structs — field iteration in declaration order plus field
names, with no macros and no code generation:

.. code-block:: cpp

   #include <rfl/to_view.hpp>

   struct Point {
     int x;
     int y;
   };

   Point p{1, 2};
   auto view = rfl::to_view(p);
   view.apply([](const auto &f) {
     // f.name() -> "x" / "y", *f.value() -> the field
   });

Only the header-only reflection core is exposed (``rfl/to_view.hpp``,
``rfl/fields.hpp``, ...). reflect-cpp's serialization formats (JSON via
yyjson, etc.) are not built and their headers should not be included on
ESP-IDF targets.

This component is the reflection backend for the reflection-driven ``cdr``
component (introduced in a follow-up PR), which uses it to have the compiler
generate CDR serialization code directly from struct definitions.

Requires C++20 or newer (the default on ESP-IDF 5.2+ toolchains).
