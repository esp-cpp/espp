#pragma once

#include "format.hpp"

// fmt formatter for espp::ThreadPool::Stats
template <> struct fmt::formatter<espp::ThreadPool::Stats> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::ThreadPool::Stats const &s, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(),
                          "ThreadPool::Stats{{submitted: {}, executed: {}, rejected: {}}}",
                          s.submitted, s.executed, s.rejected);
  }
};