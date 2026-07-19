#include <algorithm>
#include <iterator>

#include "m5stack-cardputer.hpp"

using namespace espp;

M5StackCardputer::M5StackCardputer()
    : BaseComponent("M5StackCardputer") {}

espp::Interrupt &M5StackCardputer::interrupts() { return interrupts_; }

////////////////////////
// Button Functions   //
////////////////////////

bool M5StackCardputer::initialize_button(const button_callback_t &callback) {
  logger_.info("Initializing button");
  if (button_initialized_) {
    logger_.warn("Button already initialized, not initializing again!");
    return false;
  }
  button_callback_ = callback;
  interrupts_.add_interrupt(button_interrupt_pin_);
  button_initialized_ = true;
  return true;
}

bool M5StackCardputer::button_state() const {
  if (!button_initialized_) {
    return false;
  }
  return interrupts_.is_active(button_interrupt_pin_);
}

////////////////////////
// LED Functions      //
////////////////////////

bool M5StackCardputer::initialize_led() {
  logger_.info("Initializing RGB LED");
  if (rgb_led_) {
    logger_.warn("LED already initialized, not initializing again!");
    return false;
  }
  rgb_led_ = std::make_shared<espp::Neopixel>(espp::Neopixel::Config{
      .data_gpio = rgb_led_io,
      .num_leds = 1,
  });
  return true;
}

bool M5StackCardputer::led(const Hsv &hsv) { return led(hsv.rgb()); }

bool M5StackCardputer::led(const Rgb &rgb) {
  if (!rgb_led_) {
    logger_.error("LED not initialized, you must call initialize_led() first!");
    return false;
  }
  rgb_led_->set_color(rgb);
  rgb_led_->show();
  return true;
}

////////////////////////
// ES8311 (ADV) codec //
////////////////////////

bool M5StackCardputer::es8311_write(uint8_t reg, uint8_t value) {
  if (!internal_i2c_) {
    return false;
  }
  uint8_t buffer[2] = {reg, value};
  return internal_i2c_->write(es8311_address, buffer, 2);
}

bool M5StackCardputer::es8311_ensure_common() {
  // The reset / clocking / analog-power registers are shared by the DAC
  // (speaker) and ADC (microphone) paths, and resetting the codec while the
  // other path is running would glitch it - so write them exactly once. The
  // clock manager enables both the DAC and ADC clocks (0xBF is the union of
  // M5Unified's speaker-only 0xB5 and microphone-only 0xBA values), which is
  // harmless when only one path is used.
  if (es8311_common_initialized_) {
    return true;
  }
  const uint8_t init[][2] = {
      {0x00, 0x80}, // RESET: power on, CSM enabled
      {0x01, 0xBF}, // CLOCK_MANAGER: MCLK from BCLK, DAC + ADC clocks on
      {0x02, 0x18}, // CLOCK_MANAGER: MULT_PRE = 3
      {0x0D, 0x01}, // SYSTEM: power up analog circuits
  };
  es8311_common_initialized_ =
      std::all_of(std::begin(init), std::end(init), [this](const auto &entry) {
        if (!es8311_write(entry[0], entry[1])) {
          logger_.error("Failed to write ES8311 register {:#04x}", entry[0]);
          return false;
        }
        return true;
      });
  return es8311_common_initialized_;
}

bool M5StackCardputer::initialize_es8311_speaker() {
  logger_.info("Initializing ES8311 codec (speaker / DAC path)");
  // Minimal DAC bring-up, clocked from BCLK (there is no MCLK pin); derived
  // from the M5Unified Cardputer ADV speaker-enable sequence.
  if (!es8311_ensure_common()) {
    return false;
  }
  const uint8_t init[][2] = {
      {0x12, 0x00}, // SYSTEM: power up DAC
      {0x13, 0x10}, // SYSTEM: enable output to HP drive
      {0x32, 0xBF}, // DAC: volume 0 dB
      {0x37, 0x08}, // DAC: bypass DAC equalizer
  };
  return std::all_of(std::begin(init), std::end(init), [this](const auto &entry) {
    if (!es8311_write(entry[0], entry[1])) {
      logger_.error("Failed to write ES8311 register {:#04x}", entry[0]);
      return false;
    }
    return true;
  });
}

bool M5StackCardputer::initialize_es8311_microphone() {
  logger_.info("Initializing ES8311 codec (microphone / ADC path)");
  // Minimal ADC bring-up for the analog MEMS microphone on MIC1, clocked
  // from BCLK; derived from the M5Unified Cardputer ADV microphone-enable
  // sequence.
  if (!es8311_ensure_common()) {
    return false;
  }
  // The gain values follow the es8311 driver in the espp codec component
  // (hardware-proven with the esp-box microphone): M5Unified's minimal
  // sequence uses minimum PGA gain (0x14 = 0x10) and no digital mic gain,
  // which records at a near-mute level.
  const uint8_t init[][2] = {
      {0x0E, 0x02}, // SYSTEM: enable analog PGA / ADC modulator
      {0x14, 0x1A}, // SYSTEM: select Mic1p-Mic1n, raised analog PGA gain
      {0x15, 0x40}, // ADC: soft-ramp / ALC configuration
      {0x16, 0x24}, // ADC: mic digital gain scale
      {0x17, 0xBF}, // ADC: volume 0 dB
      {0x1C, 0x6A}, // ADC: equalizer bypass
  };
  return std::all_of(std::begin(init), std::end(init), [this](const auto &entry) {
    if (!es8311_write(entry[0], entry[1])) {
      logger_.error("Failed to write ES8311 register {:#04x}", entry[0]);
      return false;
    }
    return true;
  });
}

////////////////////////
// Battery Functions  //
////////////////////////

float M5StackCardputer::battery_voltage() {
  auto maybe_mv = adc_.read_mv(battery_channel_);
  float voltage = 0;
  if (maybe_mv.has_value()) {
    voltage = maybe_mv.value() * BATTERY_VOLTAGE_SCALE;
  }
  return voltage;
}

float M5StackCardputer::battery_soc() {
  float voltage = battery_voltage();
  const auto &curve = BATTERY_SOC_CURVE;
  return piecewise_linear(curve, voltage);
}
