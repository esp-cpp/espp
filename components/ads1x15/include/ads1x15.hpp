#pragma once

#include <chrono>
#include <functional>
#include <thread>

#include "base_peripheral.hpp"

namespace espp {
/**
 * @brief Class for reading values from the ADS1x15 family of ADC chips.
 *
 * \section ads1x15_ex1 ADS1X15 Example
 * \snippet ads1x15_example.cpp ads1x15 example
 */
class Ads1x15 : public BasePeripheral<> {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = (0x48); ///< I2C address of the ADS1x15 chips.

  /**
   *  @brief Gain values for the ADC conversion.
   */
  enum class Gain {
    TWOTHIRDS = 0x0000, ///< +/-6.144V range = Gain 2/3
    ONE = 0x0200,       ///< +/-4.096V range = Gain 1
    TWO = 0x0400,       ///< +/-2.048V range = Gain 2 (default)
    FOUR = 0x0600,      ///< +/-1.024V range = Gain 4
    EIGHT = 0x0800,     ///< +/-0.512V range = Gain 8
    SIXTEEN = 0x0A00,   ///< +/-0.256V range = Gain 16
  };

  /**
   * @brief Sampling rates for the ADS1015 chips.
   */
  enum class Ads1015Rate : uint16_t {
    SPS128 = 0x0000,  ///< 128 samples per second
    SPS250 = 0x0020,  ///< 250 samples per second
    SPS490 = 0x0040,  ///< 490 samples per second
    SPS920 = 0x0060,  ///< 920 samples per second
    SPS1600 = 0x0080, ///< 1600 samples per second (default)
    SPS2400 = 0x00A0, ///< 2400 samples per second
    SPS3300 = 0x00C0, ///< 3300 samples per second
  };

  /**
   * @brief Sampling rates for the ADS1115 chips.
   */
  enum class Ads1115Rate : uint16_t {
    SPS8 = 0x0000,   ///< 8 samples per second
    SPS16 = 0x0020,  ///< 16 samples per second
    SPS32 = 0x0040,  ///< 32 samples per second
    SPS64 = 0x0060,  ///< 64 samples per second
    SPS128 = 0x0080, ///< 128 samples per second (default)
    SPS250 = 0x00A0, ///< 250 samples per second
    SPS475 = 0x00C0, ///< 475 samples per second
    SPS860 = 0x00E0, ///< 860 samples per second
  };

  /**
   * @brief Configuration for ADS1015 ADC.
   */
  struct Ads1015Config {
    uint8_t device_address = DEFAULT_ADDRESS;      ///< I2C address of the device.
    BasePeripheral::write_fn write;                ///< Function to write to the ADC
    BasePeripheral::read_fn read;                  ///< Function to read from the ADC
    Gain gain{Gain::TWOTHIRDS};                    ///< Gain for the ADC
    Ads1015Rate sample_rate{Ads1015Rate::SPS1600}; ///< Sample rate for the ADC
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Verbosity for the logger.
  };

  /**
   * @brief Configuration for ADS1115 ADC.
   */
  struct Ads1115Config {
    uint8_t device_address = DEFAULT_ADDRESS;     ///< I2C address of the device.
    BasePeripheral::write_fn write;               ///< Function to write to the ADC
    BasePeripheral::read_fn read;                 ///< Function to read from the ADC
    Gain gain{Gain::TWOTHIRDS};                   ///< Gain for the ADC
    Ads1115Rate sample_rate{Ads1115Rate::SPS128}; ///< Sample rate for the ADC
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Verbosity for the logger.
  };

  /**
   * @brief Construct Ads1x15 specficially for ADS1015.
   * @param config Configuration structure.
   */
  explicit Ads1x15(const Ads1015Config &config)
      : BasePeripheral(
            {.address = config.device_address, .write = config.write, .read = config.read},
            "Ads1015", config.log_level)
      , gain_(config.gain)
      , ads1015rate_(config.sample_rate)
      , bit_shift_(4) {}

  /**
   * @brief Construct Ads1x15 specficially for ADS1115.
   * @param config Configuration structure.
   */
  explicit Ads1x15(const Ads1115Config &config)
      : BasePeripheral(
            {.address = config.device_address, .write = config.write, .read = config.read},
            "Ads1115", config.log_level)
      , gain_(config.gain)
      , ads1115rate_(config.sample_rate)
      , bit_shift_(0) {}

  /**
   * @brief Communicate with the ADC to sample the channel and return the
   *        sampled value.
   * @param channel Which channel of the ADC to sample
   * @param ec Error code to set if there is an error.
   * @return The voltage (in mV) sampled on the channel.
   */
  float sample_mv(int channel, std::error_code &ec) {
    auto raw = sample_raw(channel, ec);
    if (ec) {
      return 0.0f;
    }
    return raw_to_mv(raw);
  }

protected:
  int16_t sample_raw(int channel, std::error_code &ec);

  bool conversion_complete(std::error_code &ec);

  float raw_to_mv(int16_t raw) const {
    // see data sheet Table 3
    float fsRange;
    switch (gain_) {
    case Gain::TWOTHIRDS:
      fsRange = 6144.0f;
      break;
    case Gain::ONE:
      fsRange = 4096.0f;
      break;
    case Gain::TWO:
      fsRange = 2048.0f;
      break;
    case Gain::FOUR:
      fsRange = 1024.0f;
      break;
    case Gain::EIGHT:
      fsRange = 512.0f;
      break;
    case Gain::SIXTEEN:
      fsRange = 256.0f;
      break;
    default:
      fsRange = 0.0f;
    }
    return raw * (fsRange / (32768 >> bit_shift_));
  }

  enum class Register : uint8_t {
    POINTER_CONVERT = 0x00,   ///< Conversion
    POINTER_CONFIG = 0x01,    ///< Configuration
    POINTER_LOWTHRESH = 0x02, ///< Low Threshold
    POINTER_HITHRESH = 0x03   ///< High Threshold
  };

  Gain gain_;
  union {
    Ads1015Rate ads1015rate_;
    Ads1115Rate ads1115rate_;
    uint16_t rate_;
  };
  int bit_shift_;
};
} // namespace espp
