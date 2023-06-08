#pragma once

#include <functional>

#include "fast_math.hpp"
#include "logger.hpp"

namespace espp {
/// @brief Thermistor class
/// @details
/// Reads voltage from a thermistor and converts it to temperature
/// using the Steinhart-Hart equation. This class is designed to be used
/// with a NTC (negative temperature coefficient) thermistor in a voltage
/// divider configuration.
/// \sa https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
///
/// \section thermistor_ex1 Validation Example
/// \snippet thermistor_example.cpp thermistor validation example
///
/// \section thermistor_ex2 ADC Example
/// \snippet thermistor_example.cpp thermistor adc example
class Thermistor {
public:
  /// @brief Function type for reading voltage
  /// @return Voltage in millivolts
  typedef std::function<float(void)> read_mv_fn;

  /// @brief Enum for resistor divider configuration
  enum class ResistorDividerConfig {
    LOWER, ///< Thermistor is the lower resistor
    UPPER, ///< Thermistor is the upper resistor
  };

  /// @brief Configuration struct for Thermistor
  struct Config {
    ResistorDividerConfig divider_config; ///< Resistor divider configuration
    float beta;                           ///< Beta coefficient of the thermistor at 25C, e.g. 3950
    float nominal_resistance_ohms; ///< Resistance of the thermistor (in ohms) at 25C, e.g. 10000
    float fixed_resistance_ohms;   ///< Resistance of the fixed resistor in the voltage divider (in
                                   ///< ohms), e.g. 10000
    float supply_mv;               ///< Supply voltage of the voltage divider (in mv), e.g. 3300
    read_mv_fn read_mv;            ///< Function for reading voltage
    Logger::Verbosity log_level = Logger::Verbosity::WARN; ///< Log level for this class
  };

  /// @brief Constructor
  /// @param config Configuration struct
  explicit Thermistor(const Config &config)
      : divider_config_(config.divider_config), b_(config.beta),
        r0_(config.nominal_resistance_ohms), r2_(config.fixed_resistance_ohms),
        supply_mv_(config.supply_mv), read_mv_(config.read_mv),
        logger_({.tag = "Thermistor", .level = config.log_level}) {
    if (b_ == 0.0f) {
      logger_.error("b_ is 0");
    } else {
      // it's not 0, so we can compute the inverse
      b_inv_ = 1.0f / b_;
    }
  }

  /// @brief Get the temperature in Kelvin
  /// @return Temperature in Kelvin
  /// @details
  /// Reads the voltage from the thermistor and converts it to temperature
  /// using the Steinhart-Hart equation.
  float get_kelvin() {
    if (read_mv_ == nullptr) {
      logger_.error("read_mv_ is null");
      return 0.0f;
    }
    if (supply_mv_ == 0.0f) {
      logger_.error("supply_mv_ is 0");
      return 0.0f;
    }
    float mv = read_mv_();
    float v_r = mv / supply_mv_; // proportion of supply voltage across the thermistor

    // compute resistance of the thermistor depending on the divider configuration
    // vout/vin = r_bottom / (r_top + r_bottom), so
    // r_bottom = r_top * (vout/vin / (1 - vout/vin)), and
    // r_top = r_bottom * (1 / vout/vin - 1)
    float r1 = 0;
    if (divider_config_ == ResistorDividerConfig::LOWER) {
      // thermistor is the lower resistor (we're computing r_bottom)
      r1 = r2_ * (v_r / (1.0f - v_r));
    } else {
      // thermistor is the upper resistor (we're computing r_top)
      r1 = r2_ * (1.0f / v_r - 1.0f);
    }

    // compute temperature using the Steinhart-Hart equation
    float t = 1.0f / (T_25C_INV + fast_ln(r1 / r0_) * b_inv_);

    logger_.debug("mv: {:.3f}, v_r: {:.3f}, r1: {:.3f}, t: {:.3f}", mv, v_r, r1, t);
    return t;
  }

  /// @brief Get the temperature in Celsius
  /// @return Temperature in Celsius
  /// @details
  /// Reads the voltage from the thermistor and converts it to temperature
  /// using the Steinhart-Hart equation.
  /// The temperature is converted from Kelvin to Celsius.
  /// @see get_kelvin()
  float get_celsius() { return get_kelvin() - T_0C; }

  /// @brief Get the temperature in Fahrenheit
  /// @return Temperature in Fahrenheit
  /// @details
  /// Reads the voltage from the thermistor and converts it to temperature
  /// using the Steinhart-Hart equation.
  /// The temperature is converted from Kelvin to Fahrenheit.
  /// @see get_kelvin()
  float get_fahrenheit() { return get_kelvin() * C_TO_F - T_0F; }

protected:
  static constexpr float C_TO_F = 1.8f;            ///< Conversion factor from Celsius to Fahrenheit
  static constexpr float T_0C = 273.15f;           ///< 0C in Kelvin
  static constexpr float T_25C = 298.15f;          ///< 25C in Kelvin
  static constexpr float T_25C_INV = 1.0f / T_25C; ///< Inverse of 25C in Kelvin (for speed)
  static constexpr float T_0F = 459.67f;           ///< Conversion offset from T_0C to Fahrenheit

  ResistorDividerConfig divider_config_; ///< Resistor divider configuration
  float b_;                              ///< Beta coefficient of the thermistor at 25C
  float b_inv_{0};                       ///< Inverse of beta coefficient (for speed)
  float r0_;                             ///< Resistance of the thermistor (in ohms) at 25C
  float r2_;                    ///< Resistance of the fixed resistor in the voltage divider
  float supply_mv_;             ///< Supply voltage of the voltage divider (in mv)
  read_mv_fn read_mv_{nullptr}; ///< Function for reading voltage (in mv)
  Logger logger_;
};
} // namespace espp
