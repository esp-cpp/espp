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

bool M5StackCardputer::initialize_es8311_speaker() {
  logger_.info("Initializing ES8311 codec (speaker path)");
  // Minimal DAC bring-up, clocked from BCLK (there is no MCLK pin); matches
  // the M5Unified Cardputer ADV speaker-enable sequence.
  const uint8_t init[][2] = {
      {0x00, 0x80}, // RESET: power on, CSM enabled
      {0x01, 0xB5}, // CLOCK_MANAGER: MCLK from BCLK
      {0x02, 0x18}, // CLOCK_MANAGER: MULT_PRE = 3
      {0x0D, 0x01}, // SYSTEM: power up analog circuits
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
  logger_.info("Initializing ES8311 codec (microphone path)");
  // Minimal ADC bring-up for the analog MEMS microphone on MIC1, clocked
  // from BCLK; matches the M5Unified Cardputer ADV microphone-enable
  // sequence.
  const uint8_t init[][2] = {
      {0x00, 0x80}, // RESET: power on, CSM enabled
      {0x01, 0xBA}, // CLOCK_MANAGER: MCLK from BCLK, ADC clocks on
      {0x02, 0x18}, // CLOCK_MANAGER: MULT_PRE = 3
      {0x0D, 0x01}, // SYSTEM: power up analog circuits
      {0x0E, 0x02}, // SYSTEM: enable analog PGA / ADC modulator
      {0x14, 0x10}, // SYSTEM: select Mic1p-Mic1n, minimum PGA gain
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
  if (voltage >= curve.front().first) {
    return curve.front().second;
  }
  if (voltage <= curve.back().first) {
    return curve.back().second;
  }
  // linearly interpolate between the two surrounding curve points
  for (size_t i = 1; i < curve.size(); i++) {
    if (voltage >= curve[i].first) {
      const auto &[v_high, soc_high] = curve[i - 1];
      const auto &[v_low, soc_low] = curve[i];
      float t = (voltage - v_low) / (v_high - v_low);
      return soc_low + t * (soc_high - soc_low);
    }
  }
  return 0.0f;
}
