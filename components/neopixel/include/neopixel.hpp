#pragma once

#include <mutex>

#include <driver/gpio.h>

#include "base_component.hpp"
#include "color.hpp"
#include "logger.hpp"
#include "rmt.hpp"

namespace espp {
/// \brief A class for controlling a Neopixel strip.
/// \details This class provides a simple interface for controlling a Neopixel
/// strip. It uses the RMT peripheral to send the data to the strip.
///
/// \section neopixel_ex1 Example 1: QtPy ESP32S3 Neopixel
/// \snippet neopixel_example.cpp neopixel ex1
class Neopixel {
public:
  /// The configuration structure for the Neopixel.
  struct Config {
    int data_gpio;      ///< The GPIO pin for the data line
    int power_gpio{-1}; ///< The GPIO pin for the power line (optional)
    size_t num_leds{1}; ///< The number of LEDs in the strip
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; ///< Verbosity for the neopixel logger
  };

  /// Constructor for the Neopixel.
  /// \param config The configuration structure for the Neopixel.
  explicit Neopixel(const espp::Neopixel::Config &config);

  /// Get the number of LEDs in the strip.
  /// \return The number of LEDs in the strip.
  std::size_t num_leds() const { return config_.num_leds; }

  /// Set the color of a single pixel.
  /// \param rgb The color to set the pixel to.
  /// \param index The index of the pixel to set.
  void set_color(const espp::Rgb &rgb, std::size_t index = 0);

  /// Set the color of a single pixel.
  /// \param hsv The color to set the pixel to.
  /// \param index The index of the pixel to set.
  void set_color(const espp::Hsv &hsv, std::size_t index = 0) { set_color(hsv.rgb(), index); }

  /// Set the color of all pixels.
  /// \param rgb The color to set all pixels to.
  void set_all(const espp::Rgb &rgb);

  /// Set the color of all pixels.
  /// \param hsv The color to set all pixels to.
  void set_all(const espp::Hsv &hsv) { set_all(hsv.rgb()); }

  /// Show the current pixel data.
  void show();

  /// Set the power state of the strip.
  /// \param on The power state to set.
  /// \details If the power GPIO is set, this function will set the power
  ///          state of the strip. If the power GPIO is not set, this function
  ///          will do nothing.
  void set_power(bool on);

protected:
  static constexpr int WS2812_FREQ_HZ = 10'000'000;
  static constexpr int MICROS_PER_SEC = 1'000'000;

  std::size_t encode(rmt_channel_handle_t channel, rmt_encoder_t *copy_encoder,
                     rmt_encoder_t *bytes_encoder, const void *data, std::size_t data_size,
                     rmt_encode_state_t *ret_state);

  void configure_power_pin();

  Config config_;
  std::shared_ptr<espp::Rmt> rmt_;
  int led_encoder_state_{0};
  std::vector<uint8_t> led_data_;
}; // class Neopixel
} // namespace espp
