#pragma once

#include <sdkconfig.h>

#if defined(CONFIG_ESPP_I2C_USE_LEGACY_API)
#include <driver/i2c.h>
#elif defined(CONFIG_ESPP_I2C_USE_NEW_API)
#include <driver/i2c_master.h>
#endif

#include "format.hpp"

// for printing of i2c_port_t with libfmt
template <> struct fmt::formatter<i2c_port_t> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext> auto format(const i2c_port_t &p, FormatContext &ctx) const {
    switch (p) {
    case I2C_NUM_0:
      return fmt::format_to(ctx.out(), "I2C_NUM_0");
    #ifdef I2C_NUM_1
      case I2C_NUM_1:
        return fmt::format_to(ctx.out(), "I2C_NUM_1");
    #endif
    default:
      return fmt::format_to(ctx.out(), "Unknown");
    }
  }
};

// for printing gpio_num_t with libfmt
template <> struct fmt::formatter<gpio_num_t> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext> auto format(const gpio_num_t &g, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "{:d}", (int)g);
  }
};

// for printing gpio_pullup_t with libfmt
template <> struct fmt::formatter<gpio_pullup_t> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext> auto format(const gpio_pullup_t &p, FormatContext &ctx) const {
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
