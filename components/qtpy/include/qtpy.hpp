#pragma once

#include <memory>
#include <string>
#include <vector>

#include "base_component.hpp"
#include "i2c.hpp"
#include "interrupt.hpp"
#include "neopixel.hpp"

namespace espp {
/// The QtPy class provides an interface to the Adafruit QtPy ESP32 and QtPy
/// ESP32-S3 development boards.
///
/// The class provides access to the following features:
/// - Button (boot button)
/// - RGB LED
/// - I2C (qwiic)
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \section qtpy_example Example
/// \snippet qtpy_example.cpp qtpy ex1
class QtPy : public BaseComponent {
public:
  /// Alias for the button callback function
  using button_callback_t = espp::Interrupt::event_callback_fn;

  /// @brief Access the singleton instance of the QtPy class
  /// @return Reference to the singleton instance of the QtPy class
  static QtPy &get() {
    static QtPy instance;
    return instance;
  }

  QtPy(const QtPy &) = delete;
  QtPy &operator=(const QtPy &) = delete;
  QtPy(QtPy &&) = delete;
  QtPy &operator=(QtPy &&) = delete;

  /// Get a reference to the interrupts
  /// \return A reference to the interrupts
  espp::Interrupt &interrupts();

  /////////////////////////////////////////////////////////////////////////////
  // Button
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the button
  /// \param callback The callback function to call when the button is pressed
  /// \return true if the button was successfully initialized, false otherwise
  bool initialize_button(const button_callback_t &callback = nullptr);

  /// Get the button state
  /// \return The button state (true = button pressed, false = button released)
  bool button_state() const;

  /////////////////////////////////////////////////////////////////////////////
  // I2C
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the qwiic I2C bus
  /// \param i2c_config The I2C configuration to use
  /// \return true if the qwiic I2C bus was successfully initialized, false
  ///         otherwise
  /// \note The SDA/SCL pins in the config will be ignored - overwritten with
  ///       the default values for the QtPy board.
  bool initialize_qwiic_i2c(const espp::I2c::Config &i2c_config = {});

  /// Get a shared pointer to the qwiic I2C bus
  /// \return A shared pointer to the qwiic I2C bus
  /// \note The qwiic I2C bus is only available if it was successfully
  ///       initialized, otherwise the shared pointer will be nullptr.
  std::shared_ptr<I2c> qwiic_i2c();

  /////////////////////////////////////////////////////////////////////////////
  // RGB LED
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the RGB LED
  /// \return true if the RGB LED was successfully initialized, false otherwise
  bool initialize_led();

  /// Get the number of LEDs in the strip
  /// \return The number of LEDs in the strip
  static constexpr size_t num_leds() { return num_leds_; }

  /// Get a shared pointer to the RGB LED
  /// \return A shared pointer to the RGB LED
  std::shared_ptr<Neopixel> led() const { return led_; }

  /// Set the color of the LED
  /// \param hsv The color of the LED in HSV format
  /// \return true if the color was successfully set, false otherwise
  bool led(const Hsv &hsv);

  /// Set the color of the LED
  /// \param rgb The color of the LED in RGB format
  /// \return true if the color was successfully set, false otherwise
  bool led(const Rgb &rgb);

protected:
  QtPy();

#if CONFIG_IDF_TARGET_ESP32
  static constexpr bool IS_ESP32 = true;
  static constexpr bool IS_ESP32_S3 = false;

  // LED
  static constexpr auto led_data_io = GPIO_NUM_5;
  static constexpr auto led_power_io = GPIO_NUM_8;

  // I2C
  static constexpr auto qwiic_sda_io = GPIO_NUM_22;
  static constexpr auto qwiic_scl_io = GPIO_NUM_19;
#elif CONFIG_IDF_TARGET_ESP32S3
  static constexpr bool IS_ESP32 = false;
  static constexpr bool IS_ESP32_S3 = true;

  // LED
  static constexpr auto led_data_io = GPIO_NUM_39;
  static constexpr auto led_power_io = GPIO_NUM_38;

  // I2C
  static constexpr auto qwiic_sda_io = GPIO_NUM_41;
  static constexpr auto qwiic_scl_io = GPIO_NUM_40;
#else
#error "Unsupported target"
#endif

  // button (boot button)
  static constexpr gpio_num_t button_io = GPIO_NUM_0; // active low

  // common:
  // WS2812 LED
  static constexpr auto led_clock_speed = 10 * 1000 * 1000; // 10 MHz
  static constexpr auto num_leds_ = 1;

  // Interrupts
  espp::Interrupt::PinConfig button_interrupt_pin_{
      .gpio_num = button_io,
      .callback =
          [this](const auto &event) {
            if (button_callback_) {
              button_callback_(event);
            }
          },
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
      .pullup_enabled = true};

  // we'll only add each interrupt pin if the initialize method is called
  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "qtpy interrupts",
                       .stack_size_bytes = CONFIG_QTPY_INTERRUPT_STACK_SIZE}}};

  // button
  std::atomic<bool> button_initialized_{false};
  button_callback_t button_callback_{nullptr};

  // led
  std::shared_ptr<Neopixel> led_;

  // I2C
  std::shared_ptr<I2c> qwiic_i2c_;
}; // class QtPy
} // namespace espp
