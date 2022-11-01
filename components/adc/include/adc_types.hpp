#pragma once

namespace espp {
  /**
   * @brief Configuration structure for an ADC channel.
   */
  struct AdcConfig {
    adc_unit_t unit; /**< Which adc unit is this channel associated with, e.g. ADC_UNIT_1. */
    adc_channel_t channel; /**< The actual channel, e.g. ADC_CHANNEL_2. */
    adc_atten_t attenuation; /**< The attenuation associated with this channel, e.g. ADC_ATTEN_DB_11. */
  };
}
