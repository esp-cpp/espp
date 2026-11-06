#include <algorithm>
#include <cmath>
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

// register access for CardputerAdvCodec (src/cardputer_adv_codec.hpp)
bool M5StackCardputer::es8311_write(uint8_t reg, uint8_t value) {
  if (!internal_i2c_) {
    return false;
  }
  uint8_t buffer[2] = {reg, value};
  return internal_i2c_->write(es8311_address, buffer, 2);
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
