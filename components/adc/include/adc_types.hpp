#pragma once

#include "format.hpp"

#include <hal/adc_types.h>

namespace espp {
/**
 * @brief Configuration structure for an ADC channel.
 */
struct AdcConfig {
  adc_unit_t unit;       /**< Which adc unit is this channel associated with, e.g. ADC_UNIT_1. */
  adc_channel_t channel; /**< The actual channel, e.g. ADC_CHANNEL_2. */
  adc_atten_t
      attenuation; /**< The attenuation associated with this channel, e.g. ADC_ATTEN_DB_11. */
};

static bool operator!=(const AdcConfig &lhs, const AdcConfig &rhs) {
  return lhs.unit != rhs.unit || lhs.channel != rhs.channel || lhs.attenuation != rhs.attenuation;
}

static bool operator==(const AdcConfig &lhs, const AdcConfig &rhs) { return !(lhs != rhs); }
} // namespace espp

// for libfmt printing of adc_unit_t
template <> struct fmt::formatter<adc_unit_t> : fmt::formatter<std::string> {
  template <typename FormatContext> auto format(adc_unit_t t, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "ADC_UNIT_{}", (int)t + 1);
  }
};

// for libfmt printing of adc_channel_t
template <> struct fmt::formatter<adc_channel_t> : fmt::formatter<std::string> {
  template <typename FormatContext> auto format(adc_channel_t t, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "ADC_CHANNEL_{}", (int)t);
  }
};

// for libfmt printing of adc_atten_t
template <> struct fmt::formatter<adc_atten_t> : fmt::formatter<std::string> {
  template <typename FormatContext> auto format(adc_atten_t t, FormatContext &ctx) const {
    switch (t) {
    case ADC_ATTEN_DB_0:
      return fmt::format_to(ctx.out(), "ADC_ATTEN_DB_0");
    case ADC_ATTEN_DB_2_5:
      return fmt::format_to(ctx.out(), "ADC_ATTEN_DB_2_5");
    case ADC_ATTEN_DB_6:
      return fmt::format_to(ctx.out(), "ADC_ATTEN_DB_6");
    case ADC_ATTEN_DB_12:
      return fmt::format_to(ctx.out(), "ADC_ATTEN_DB_12");
    }
    return fmt::format_to(ctx.out(), "ADC_ATTEN_UNKNOWN");
  }
};

// for easy serialization of AdcConfig with libfmt
template <> struct fmt::formatter<espp::AdcConfig> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const espp::AdcConfig &c, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "AdcConfig(unit={}, channel={}, attenuation={})", c.unit,
                          c.channel, c.attenuation);
  }
};
