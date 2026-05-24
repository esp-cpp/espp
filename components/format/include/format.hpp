#pragma once

#include <string>

#define FMT_HEADER_ONLY
#include <fmt/base.h>
#include <fmt/format.h>

#include <fmt/chrono.h>
#include <fmt/color.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>
#include <fmt/std.h>

#if !FMT_USE_INT128
namespace fmt {
inline namespace v12 {
namespace detail {
constexpr auto operator~(const uint128 &value) -> uint128 { return {~value.high(), ~value.low()}; }
} // namespace detail
} // namespace v12
} // namespace fmt
#endif
