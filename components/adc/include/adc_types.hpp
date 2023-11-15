#pragma once

#include "format.hpp"

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

// for easy serialization of AdcConfig with libfmt
template <> struct fmt::formatter<espp::AdcConfig> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext> auto format(const espp::AdcConfig &c, FormatContext &ctx) {
    return fmt::format_to(ctx.out(), "AdcConfig(unit={}, channel={}, attenuation={})", (int)c.unit,
                          (int)c.channel, (int)c.attenuation);
  }
};
