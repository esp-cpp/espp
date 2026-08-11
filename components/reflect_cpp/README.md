# reflect_cpp

Vendors [getml/reflect-cpp](https://github.com/getml/reflect-cpp) (pinned to
`v0.25.0` as a git submodule under `detail/`) so espp components can use
compile-time reflection over plain aggregate structs — field iteration in
declaration order plus field names, with no macros and no code generation:

```cpp
#include <rfl/to_view.hpp>

struct Point { int x; int y; };

Point p{1, 2};
auto view = rfl::to_view(p);
view.apply([](const auto &f) {
  // f.name() -> "x" / "y", f.value() -> pointer to the field
});
```

Only the header-only reflection core is exposed (`rfl/to_view.hpp`,
`rfl/fields.hpp`, ...). reflect-cpp's serialization formats (JSON via
yyjson, etc.) are not built and their headers should not be included on
ESP-IDF targets.

Requires C++20 or newer (ESP-IDF 5.2+ toolchains). The reflection-driven
`cdr` component (stacked follow-up PR) uses this component as its backend.

When updating the submodule pin, keep it on the same reflect-cpp tag that
[finger563/cdr](https://github.com/finger563/cdr) fetches in its standalone
build (see that repo's `CMakeLists.txt`).
