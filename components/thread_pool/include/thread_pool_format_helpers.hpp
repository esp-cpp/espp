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
                          "ThreadPool::Stats{{submitted: {}, executed: {}, rejected: {}, "
                          "band_submitted: [{}, {}, {}, {}], band_executed: [{}, {}, {}, {}], "
                          "band_aged: [{}, {}, {}, {}]}}",
                          s.submitted, s.executed, s.rejected, s.band_submitted[0],
                          s.band_submitted[1], s.band_submitted[2], s.band_submitted[3],
                          s.band_executed[0], s.band_executed[1], s.band_executed[2],
                          s.band_executed[3], s.band_aged[0], s.band_aged[1], s.band_aged[2],
                          s.band_aged[3]);
  }
};
