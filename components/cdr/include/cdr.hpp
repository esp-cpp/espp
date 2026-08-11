#pragma once

/// @file cdr.hpp
/// @brief espp umbrella header for the cdr serialization library.
///
/// Reflection-driven CDR/XCDR serialization for plain C++ structs — the
/// compiler generates the serialization code from the struct definition
/// itself; there is no IDL compiler and no hand-written read/write sequence.
/// The full library lives at https://github.com/finger563/cdr (vendored under
/// detail/); see its docs/DESIGN.md for the architecture and wire-format
/// rules, and README.md for the type mapping and usage conventions.
///
/// Everything is in the `cdr` namespace (not `espp`), matching the standalone
/// library:
///
/// \code{.cpp}
/// struct ImuSample {
///   uint64_t stamp_us;
///   std::array<float, 3> accel;
/// };
///
/// auto bytes = cdr::serialize(sample);              // XCDR2, appendable
/// auto ros2  = cdr::serialize<cdr::xcdr1>(sample);  // ROS 2 / classic CDR
/// auto back  = cdr::deserialize<ImuSample>(*bytes); // std::expected
/// \endcode
///
/// Requires C++23 (std::expected) — the default C++ standard on ESP-IDF's
/// GCC 13+ toolchains (IDF 5.2 and newer).

#include <cdr/cdr.hpp>
