#pragma once

#include <driver/i2c.h>

#include "format.hpp"

// for printing of i2c_port_t with libfmt
template <> struct fmt::formatter<i2c_port_t> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext> auto format(const i2c_port_t &p, FormatContext &ctx) {
    switch (p) {
    case I2C_NUM_0:
      return fmt::format_to(ctx.out(), "I2C_NUM_0");
    case I2C_NUM_1:
      return fmt::format_to(ctx.out(), "I2C_NUM_1");
    default:
      return fmt::format_to(ctx.out(), "Unknown");
    }
  }
};
