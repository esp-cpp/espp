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

// for printing gpio_num_t with libfmt
template <> struct fmt::formatter<gpio_num_t> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext> auto format(const gpio_num_t &g, FormatContext &ctx) {
    return fmt::format_to(ctx.out(), "{:d}", (int)g);
  }
};

// for printing gpio_pullup_t with libfmt
template <> struct fmt::formatter<gpio_pullup_t> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext> auto format(const gpio_pullup_t &p, FormatContext &ctx) {
    switch (p) {
    case GPIO_PULLUP_DISABLE:
      return fmt::format_to(ctx.out(), "GPIO_PULLUP_DISABLE");
    case GPIO_PULLUP_ENABLE:
      return fmt::format_to(ctx.out(), "GPIO_PULLUP_ENABLE");
    default:
      return fmt::format_to(ctx.out(), "Unknown");
    }
  }
};
