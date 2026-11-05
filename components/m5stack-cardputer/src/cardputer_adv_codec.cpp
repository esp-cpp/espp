#include "cardputer_adv_codec.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <esp_log.h>

using namespace espp;

static const char *TAG = "CardputerAdvCodec";

static uint8_t adc_volume_from_gain(float percent) {
  return static_cast<uint8_t>(std::lround(std::clamp(percent, 0.0f, 100.0f) / 100.0f * 255.0f));
}

bool CardputerAdvCodec::write_all(std::initializer_list<std::pair<uint8_t, uint8_t>> registers) {
  for (auto [reg, value] : registers) {
    if (!write_ || !write_(reg, value)) {
      ESP_LOGE(TAG, "Failed to write ES8311 register 0x%02x", reg);
      return false;
    }
  }
  return true;
}

bool CardputerAdvCodec::start(Direction direction, uint32_t sample_rate) {
  direction_ = direction;
  const bool dac = direction == Direction::PLAYBACK || direction == Direction::BOTH;
  const bool adc = direction == Direction::RECORDING || direction == Direction::BOTH;
  ESP_LOGI(TAG, "Initializing the ES8311 (%s%s%s) at %u Hz", dac ? "DAC" : "",
           dac && adc ? " + " : "", adc ? "ADC" : "", static_cast<unsigned>(sample_rate));
  // The clock manager enables both the DAC and ADC clocks (0xBF is the union
  // of M5Unified's speaker-only 0xB5 and microphone-only 0xBA values), which
  // is harmless when only one path is used.
  if (!write_all({
          {0x00, 0x80}, // RESET: power on, CSM enabled
          {0x01, 0xBF}, // CLOCK_MANAGER: MCLK from BCLK, DAC + ADC clocks on
          {0x02, 0x18}, // CLOCK_MANAGER: MULT_PRE = 3
          {0x0D, 0x01}, // SYSTEM: power up analog circuits
      })) {
    return false;
  }
  if (dac && !write_all({
                 {0x12, 0x00}, // SYSTEM: power up DAC
                 {0x13, 0x10}, // SYSTEM: enable output to HP drive
                 {0x32, 0xBF}, // DAC: volume 0 dB
                 {0x37, 0x08}, // DAC: bypass DAC equalizer
             })) {
    return false;
  }
  // The gain values follow the es8311 driver in the espp codec component
  // (hardware-proven with the esp-box microphone): M5Unified's minimal
  // sequence uses minimum PGA gain (0x14 = 0x10) and no digital mic gain,
  // which records at a near-mute level.
  if (adc && !write_all({
                 {0x0E, 0x02},                        // SYSTEM: enable analog PGA / ADC modulator
                 {0x14, 0x1A},                        // SYSTEM: select Mic1p-Mic1n, raised PGA gain
                 {0x15, 0x40},                        // ADC: soft-ramp / ALC configuration
                 {0x16, 0x24},                        // ADC: mic digital gain scale
                 {0x17, adc_volume_from_gain(gain_)}, // ADC: volume
                 {0x1C, 0x6A},                        // ADC: equalizer bypass
             })) {
    return false;
  }
  return true;
}

void CardputerAdvCodec::stop() {
  // power down what start() powered up (the ES8311 driver's suspend values;
  // start() writes each of these registers again)
  const bool dac = direction_ == Direction::PLAYBACK || direction_ == Direction::BOTH;
  const bool adc = direction_ == Direction::RECORDING || direction_ == Direction::BOTH;
  if (dac) {
    write_all({
        {0x32, 0x00}, // DAC: volume -95.5 dB
        {0x12, 0x02}, // SYSTEM: power down DAC
    });
  }
  if (adc) {
    write_all({
        {0x17, 0x00}, // ADC: volume -95.5 dB
        {0x0E, 0xFF}, // SYSTEM: power down PGA / ADC modulator
        {0x14, 0x00}, // SYSTEM: no microphone input
        {0x15, 0x00}, // ADC: ramp / ALC off
    });
  }
  write_all({{0x0D, 0xFA}}); // SYSTEM: power down analog circuits
}

bool CardputerAdvCodec::set_gain(float percent) {
  gain_ = std::clamp(percent, 0.0f, 100.0f);
  return write_all({{0x17, adc_volume_from_gain(gain_)}}); // ADC: volume
}
