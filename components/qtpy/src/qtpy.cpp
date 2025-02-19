#include "qtpy.hpp"

using namespace espp;

QtPy::QtPy()
    : BaseComponent(IS_ESP32 ? "QtPy Pico" : "QtPy ESP32-S3") {}

std::shared_ptr<espp::I2c> QtPy::qwiic_i2c() { return qwiic_i2c_; }

espp::Interrupt &QtPy::interrupts() { return interrupts_; }

////////////////////////
// Button Functions   //
////////////////////////

bool QtPy::initialize_button(const QtPy::button_callback_t &callback) {
  logger_.info("Initializing button");

  // save the callback
  button_callback_ = callback;

  // configure the button
  interrupts_.add_interrupt(button_interrupt_pin_);
  button_initialized_ = true;
  return true;
}

bool QtPy::button_state() const {
  if (!button_initialized_) {
    return false;
  }
  return interrupts_.is_active(button_interrupt_pin_);
}

////////////////////////
// I2C Functions      //
////////////////////////

bool QtPy::initialize_qwiic_i2c(const espp::I2c::Config &i2c_config) {
  if (qwiic_i2c_) {
    logger_.warn("Qwiic I2C already initialized");
    return false;
  }

  logger_.info("Initializing Qwiic I2C");
  auto config = i2c_config;
  config.sda_io_num = qwiic_sda_io;
  config.scl_io_num = qwiic_scl_io;

  qwiic_i2c_ = std::make_shared<espp::I2c>(config);

  return true;
}

////////////////////////
// LED Functions      //
////////////////////////

bool QtPy::initialize_led() {
  if (led_) {
    logger_.warn("LED already initialized");
    return false;
  }

  logger_.info("Initializing LED");
  led_ = std::make_shared<espp::Neopixel>(
      espp::Neopixel::Config{.data_gpio = led_data_io, .power_gpio = led_power_io});
  return true;
}

/// Set the color of the LED
/// \param hsv The color of the LED in HSV format
/// \return True if the LED was set, false otherwise
bool QtPy::led(const Hsv &hsv) { return led(hsv.rgb()); }

/// Set the color of the LED
/// \param rgb The color of the LED in RGB format
/// \return True if the LED was set, false otherwise
bool QtPy::led(const Rgb &rgb) {
  if (led_) {
    led_->set_color(rgb);
    led_->show();
    return true;
  }
  return false;
}
