#pragma once

#include "format.hpp"

// for allowing easy serialization/printing of the
// espp::Socket::Info
template <> struct fmt::formatter<espp::Socket::Info> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::Socket::Info const &info, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "{}:{}", info.address, info.port);
  }
};
