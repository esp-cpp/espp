#pragma once

#include <functional>
#include <thread>

#include "base_peripheral.hpp"

namespace espp {
using vl53l_register_t = uint16_t;

/// \brief VL53LXXX Time-of-Flight Distance Sensor
/// \details
/// The VL53LXX is a new generation Time-of-Flight (ToF) laser-ranging module
/// housed in the smallest package on the market today, providing accurate
/// distance measurement whatever the target reflectances unlike conventional
/// technologies. It can measure absolute distances up to 2m, setting a new
/// benchmark in ranging performance levels, opening the door to various new
/// applications.
/// \see https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html
/// \see https://www.st.com/en/imaging-and-photonics-solutions/vl53l4CD.html
/// \see https://www.st.com/en/imaging-and-photonics-solutions/vl53l4CX.html
///
/// \note
/// For an official Arduino library, see
/// https://github.com/stm32duino/VL53L4CX
///
/// \see https://github.com/adafruit/Adafruit_CircuitPython_VL53L4CD/tree/main
/// for a CircuitPython library
///
/// \section vl53l_ex1 Example
/// \snippet vl53l_example.cpp vl53l example
class Vl53l : public espp::BasePeripheral<vl53l_register_t, true> {
  using BasePeripheral<vl53l_register_t, true>::set_address;
  using BasePeripheral<vl53l_register_t, true>::set_write;
  using BasePeripheral<vl53l_register_t, true>::set_read;
  using BasePeripheral<vl53l_register_t, true>::write_u8_to_register;
  using BasePeripheral<vl53l_register_t, true>::write_u16_to_register;
  using BasePeripheral<vl53l_register_t, true>::write_many_to_register;
  using BasePeripheral<vl53l_register_t, true>::read_u8_from_register;
  using BasePeripheral<vl53l_register_t, true>::read_u16_from_register;
  using BasePeripheral<vl53l_register_t, true>::read_many_from_register;
  using BasePeripheral<vl53l_register_t, true>::clear_bits_in_register;
  using BasePeripheral<vl53l_register_t, true>::set_bits_in_register;
  using BasePeripheral<vl53l_register_t, true>::set_bits_in_register_by_mask;
  using BasePeripheral<vl53l_register_t, true>::read;
  using BasePeripheral<vl53l_register_t, true>::logger_;

public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x29; // 7-bit address (decimal: 41)

  /// \brief Model ID and module type
  struct ModelInfo {
    uint8_t model_id{0};    ///< Model ID
    uint8_t module_type{0}; ///< Module type
  };

  /// \brief Distance mode
  enum class DistanceMode {
    SHORT = 1,  ///< Short distance mode
    MEDIUM = 2, ///< Medium distance mode
    LONG = 3,   ///< Long distance mode
  };

  /// \brief Range status
  enum class RangeStatus {
    VALID = 0,                        ///< Range valid
    SIGMA_FAIL = 1,                   ///< Sigma fail
    SIGNAL_FAIL = 2,                  ///< Signal fail
    BELOW_RANGE = 3,                  ///< Below range
    OUT_OF_BOUNDS_FAIL = 4,           ///< Out of bounds fail
    HARDWARE_FAIL = 5,                ///< Hardware fail
    VALID_NO_WRAP_CHECK_FAIL = 6,     ///< Valid, but wraparound check has not been done
    NO_MATCHING_PHASE = 7,            ///< No matching phase
    PROCESSING_FAIL = 8,              ///< Processing fail
    XTALK_SIGNAL_FAIL = 9,            ///< Crosstalk signal fail
    SYNCRONISATION_INT = 10,          ///< Synchronisation interrupt (ignore data)
    RANGE_IGNORE_THRESHOLD_FAIL = 11, ///< Range ignore threshold fail
  };

  /// \brief Range data
  struct RangeData {
    int16_t max_mm;          ///< Maximum distance in millimeters
    int16_t min_mm;          ///< Minimum distance in millimeters
    float return_rate_mcps;  ///< Return rate in MCPS (mega counts per second)
    float ambient_rate_mcps; ///< Ambient rate in MCPS (mega counts per second)
    float sigma_mm;          ///< Sigma in millimeters
    int16_t range_mm;        ///< Range in millimeters
    bool valid;              // NOTE: raw value of 0 means range is valid
    bool extended; // NOTE: raw value of 1 means timings A&B are combined to give extended range
  };

  /// \brief Configuration for the VL53LXX
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address of the VL53LXX
    BasePeripheral<vl53l_register_t, true>::write_fn write = nullptr; ///< Write function
    BasePeripheral<vl53l_register_t, true>::read_fn read = nullptr;   ///< Read function
    bool auto_init = true;                                 ///< Automatically initialize the sensor
    Logger::Verbosity log_level = Logger::Verbosity::WARN; ///< Log level for this class
  };

  /// \brief Constructor
  /// \param config Configuration
  /// \see Config
  explicit Vl53l(const Config &config)
      : BasePeripheral<vl53l_register_t, true>({}, "VL53LXX", config.log_level) {
    set_address(config.device_address);
    set_write(config.write);
    set_read(config.read);
    if (config.auto_init) {
      std::error_code ec;
      init(config, ec);
    }
  }

  /// \brief Get the ModelInfo
  /// \param ec Error code if unsuccessful
  /// \return ModelInfo
  /// \see VL53L0X_GetModelID
  ModelInfo get_model_info(std::error_code &ec) {
    uint8_t data[2];
    if (!read_reg(Register::IDENTIFICATION_MODEL_ID, data, 2, ec)) {
      return {};
    }
    ModelInfo info;
    info.model_id = data[0];
    info.module_type = data[1];
    logger_.debug("Model ID: {:#04x}, Module Type: {:#04x}", info.model_id, info.module_type);
    if (info.model_id != MODEL_ID || info.module_type != MODULE_TYPE) {
      logger_.error("Model ID ({:#04x}) or Module Type ({:#04x}) does not match expected values "
                    "({:#04x}, {:#04x})",
                    info.model_id, info.module_type, MODEL_ID, MODULE_TYPE);
    }
    return info;
  }

  /// \brief Get the current distance in millimeters
  /// \param ec Error code if unsuccessful
  /// \return Distance in millimeters
  uint16_t get_distance_mm(std::error_code &ec) {
    uint8_t data[2];
    if (!read_reg(Register::RESULT_DISTANCE_MM, data, 2, ec)) {
      return 0;
    }
    return (data[0] << 8) | data[1];
  }

  /// \brief Get the current distance in centimeters
  /// \param ec Error code if unsuccessful
  /// \return Distance in centimeters
  float get_distance_cm(std::error_code &ec) { return get_distance_mm(ec) / 10.0f; }

  /// \brief Get the current distance in meters
  /// \param ec Error code if unsuccessful
  /// \return Distance in meters
  float get_distance_meters(std::error_code &ec) { return get_distance_mm(ec) / 1000.0f; }

  /// \brief Get the inter-measurement period in milliseconds
  /// \return Inter-measurement period in milliseconds
  int get_inter_measurement_period_ms() const { return inter_measurement_period_ms_; }

  /// \brief Read the current inter-measurement time in milliseconds
  /// \details
  /// The inter-measurement period determines how often the sensor takes a
  /// measurement. Valid range is timing_budget to 5000 ms. Set to 0 to
  /// disable continuous mode.
  /// \param ec Error code if unsuccessful
  /// \return Inter-measurement time in milliseconds
  int read_inter_measurement_period_ms(std::error_code &ec) {
    logger_.debug("Getting inter-measurement period");
    // read the intermeasurement_ms register (4 bytes)
    uint8_t data[4];
    if (!read_reg(Register::INTERMEASUREMENT_MS, data, 4, ec)) {
      return 0;
    }
    uint32_t reg_val = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    // read the OSC_CALIBRATE register (2 bytes) to get the clock pll
    if (!read_reg(Register::OSC_CALIBRATE_VAL, data, 2, ec)) {
      return 0;
    }
    // mask out the clock pll bits (0x3FF)
    uint16_t osc_calibrate_val = (data[0] << 8) | data[1];
    uint16_t clock_pll = osc_calibrate_val & 0x3FF;
    // multiply the clock pll by 1.065
    float factor = ((float)clock_pll * 1.065f);
    int inter_measurement_ms = (int)(reg_val / factor);
    logger_.debug("Inter-measurement period is {} ms", inter_measurement_ms);
    // store the inter-measurement period
    inter_measurement_period_ms_ = inter_measurement_ms;
    return inter_measurement_ms;
  }

  /// \brief Set the inter-measurement time in milliseconds
  /// \details
  /// The inter-measurement period determines how often the sensor takes a
  /// measurement. Valid range is timing_budget to 5000 ms. Set to 0 to
  /// disable continuous mode.
  /// \param period_ms Inter-measurement time in milliseconds
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  bool set_inter_measurement_period_ms(int period_ms, std::error_code &ec) {
    if (ranging_) {
      logger_.error("Cannot set inter-measurement period while ranging. Stop ranging first.");
      ec = make_error_code(std::errc::device_or_resource_busy);
      return false;
    }

    logger_.debug("Setting inter-measurement period to {} ms", period_ms);

    if (period_ms < 0) {
      period_ms = 0;
    }

    if (period_ms == 0) {
      // we're in continuous mode, so save that state
      continuous_ = true;
    } else {
      // we're in autonomous mode, so save that state
      continuous_ = false;
    }

    // make sure the inter-measurement period is at least the timing budget
    int timing_budget_ms = get_timing_budget_ms();
    if (period_ms != 0 && period_ms < timing_budget_ms) {
      logger_.warn("Inter-measurement period ({} ms) is less than timing budget ({} ms). Setting "
                   "to timing budget.",
                   period_ms, timing_budget_ms);
      period_ms = timing_budget_ms;
    }

    // read the OSC_CALIBRATE register (2 bytes) to get the clock pll
    uint8_t data[4];
    if (!read_reg(Register::OSC_CALIBRATE_VAL, data, 2, ec)) {
      return false;
    }
    // mask out the clock pll bits (0x3FF)
    uint16_t osc_calibrate_val = (data[0] << 8) | data[1];
    uint16_t clock_pll = osc_calibrate_val & 0x3FF;
    // multiply the clock pll by 1.065
    float factor = ((float)clock_pll * 1.055f);
    // divide the period by the factor
    uint32_t reg_val = (uint32_t)((float)period_ms * factor);
    // write the intermeasurement_ms register (4 bytes)
    data[0] = (reg_val >> 24) & 0xFF;
    data[1] = (reg_val >> 16) & 0xFF;
    data[2] = (reg_val >> 8) & 0xFF;
    data[3] = reg_val & 0xFF;
    if (!write_reg(Register::INTERMEASUREMENT_MS, data, 4, ec)) {
      logger_.error("Failed to set inter-measurement period to {} ms", period_ms);
      return false;
    }
    logger_.debug("Inter-measurement period set to {} ms", period_ms);
    // store the inter-measurement period
    inter_measurement_period_ms_ = period_ms;
    return true;
  }

  /// \brief Start ranging
  /// \details
  /// Ranging depends on the inter-measurement setting (continuous mode vs
  /// autonomous mode). In continuous mode, the sensor will take measurements
  /// continuously. In autonomous mode, the sensor will take a single
  /// measurement each time start_ranging() is called.
  /// \see stop_ranging()
  /// \see set_inter_measurement_period_ms()
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  bool start_ranging(std::error_code &ec) {
    logger_.debug("Starting ranging");
    // ranging depends on inter-measurement setting (continuous mode vs
    // autonomous mode)
    uint8_t start_byte = continuous_ ? 0x21 : 0x40;
    if (!write_reg(Register::SYSTEM_START, start_byte, ec)) {
      return false;
    }

    logger_.debug("Waiting for data ready");
    // wait for data ready
    while (!is_data_ready(ec)) {
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(1ms);
    }

    // clear interrupts
    if (!clear_interrupt(ec)) {
      return false;
    }
    // store ranging_ = true
    ranging_ = true;
    return true;
  }

  /// \brief Stop ranging
  /// \details
  /// Stop ranging. This will put the sensor in standby mode.
  /// \see start_ranging()
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  bool stop_ranging(std::error_code &ec) {
    logger_.debug("Stopping ranging");
    if (!write_reg(Register::SYSTEM_START, 0x00, ec)) {
      return false;
    }
    ranging_ = false;
    return true;
  }

  /// \brief Is data ready
  /// \details
  /// Check if data is ready to be read. This is used to determine if a
  /// measurement is available to be read.
  /// \param ec Error code if unsuccessful
  /// \return True if data is ready
  bool is_data_ready(std::error_code &ec) {
    // read the GPIO_TIO_HV_STATUS register and return true if the first bit
    // is equal to the interrupt polarity
    uint8_t val;
    if (!read_reg(Register::GPIO_TIO_HV_STATUS, &val, 1, ec)) {
      return false;
    }
    val = val & 0x01;
    logger_.debug("Data ready: {}", val);
    return (bool)val == get_interrupt_polarity();
  }

  /// \brief Clear the interrupt
  /// \details
  /// Clear the interrupt. This is called automatically by start_ranging().
  /// \see start_ranging()
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  bool clear_interrupt(std::error_code &ec) {
    return write_reg(Register::SYSTEM_INTERRUPT_CLEAR, 0x01, ec);
  }

  /// \brief Get the current timing budget in milliseconds
  /// \return Timing budget in milliseconds
  int get_timing_budget_ms() const { return timing_budget_us_ / 1000; }

  /// \brief Get the current timing budget in microseconds
  /// \return Timing budget in microseconds
  uint32_t get_timing_budget_us() const { return timing_budget_us_; }

  /// \brief Read the current timing budget in microseconds
  /// \details
  /// The timing budget is the time allowed for one measurement. Valid range
  /// is 10ms to 200ms. It cannot be greater than the inter-measurement
  /// period.
  /// \param ec Error code if unsuccessful
  /// \return Timing budget in microseconds
  /// \see set_timing_budget_us()
  /// \see set_inter_measurement_period_ms()
  /// \see get_inter_measurement_period_ms()
  uint32_t read_timing_budget_ms(std::error_code &ec) {
    logger_.info("Getting timing budget");

    // NOTE: I have no idea what the code below is doing...

    // get the oscillator frequency (1 byte from register 0x0006)
    uint8_t data[2];
    if (!read_reg(Register::OSCILLATOR_FREQUENCY, data, 2, ec)) {
      return 0;
    }
    uint16_t oscillator_frequency = (data[0] << 8) | data[1];

    // calculate the macro period (us) from the oscillator frequency
    uint32_t macro_period_us = 16 * (int(2304 * (0x40000000 / oscillator_frequency)) >> 6);

    uint8_t macrop_high;
    if (!read_reg(Register::RANGE_CONFIG_A, &macrop_high, 1, ec)) {
      return 0;
    }

    uint8_t ls_byte = (macrop_high & 0x00FF) << 4;
    uint8_t ms_byte = (macrop_high & 0xFF00) >> 8;
    ms_byte = 0x04 - (ms_byte - 1) - 1; // intentionally allowed underflow

    int timing_budget_us =
        (((ls_byte + 1) * (macro_period_us >> 6)) - ((macro_period_us >> 6) >> 1)) >> 12;
    // cppcheck-suppress knownConditionTrueFalse
    if (ms_byte < 12) {
      timing_budget_us >>= ms_byte;
    }
    auto intermeasurement_period_ms = get_inter_measurement_period_ms();
    if (ec) {
      return 0;
    }
    if (intermeasurement_period_ms == 0) {
      // we're in continuous mode
      timing_budget_us += 2500;
    } else {
      // we're in autonomous mode
      timing_budget_us *= 2;
      timing_budget_us += 4300;
    }

    logger_.debug("Timing budget is {} us", timing_budget_us);
    // store the timing budget
    timing_budget_us_ = timing_budget_us;
    return timing_budget_us / 1000;
  }

  /// \brief Get the current timing budget in seconds
  /// \details
  /// The timing budget is the time allowed for one measurement. Valid range
  /// is 10ms to 200ms. It cannot be greater than the inter-measurement
  /// period.
  /// \return Timing budget in seconds
  /// \see set_timing_budget_us()
  /// \see set_inter_measurement_period_ms()
  /// \see get_inter_measurement_period_ms()
  float get_timing_budget_seconds() { return get_timing_budget_ms() / 1000.0f; }

  /// \brief Read the current timing budget in seconds
  /// \details
  /// The timing budget is the time allowed for one measurement. Valid range
  /// is 10ms to 200ms. It cannot be greater than the inter-measurement
  /// period.
  /// \param ec Error code if unsuccessful
  /// \return Timing budget in seconds
  /// \see set_timing_budget_us()
  /// \see set_inter_measurement_period_ms()
  /// \see get_inter_measurement_period_ms()
  float read_timing_budget_seconds(std::error_code &ec) {
    return read_timing_budget_ms(ec) / 1000.0f;
  }

  /// \brief Set the timing budget in microseconds
  /// \details
  /// The timing budget is the time allowed for one measurement. Valid range
  /// is 10ms to 200ms. It cannot be greater than the inter-measurement
  /// period.
  /// \param budget_us Timing budget in microseconds
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  /// \see get_timing_budget_us()
  /// \see set_inter_measurement_period_ms()
  /// \see get_inter_measurement_period_ms()
  bool set_timing_budget_us(uint32_t budget_us, std::error_code &ec) {
    if (ranging_) {
      logger_.error("Cannot set timing budget while ranging.");
      ec = make_error_code(std::errc::device_or_resource_busy);
      return false;
    }

    if (budget_us < MIN_TIMING_BUDGET_US || budget_us > MAX_TIMING_BUDGET_US) {
      logger_.error("Invalid timing budget {}, must be between {} and {}.", budget_us,
                    MIN_TIMING_BUDGET_US, MAX_TIMING_BUDGET_US);
      ec = make_error_code(std::errc::invalid_argument);
      return false;
    }

    auto intermeasurement_period_ms = get_inter_measurement_period_ms();
    if (intermeasurement_period_ms != 0 && budget_us > intermeasurement_period_ms * 1000) {
      logger_.error("Invalid timing budget {}, must be less than inter-measurement period {}.",
                    budget_us, intermeasurement_period_ms * 1000);
      ec = make_error_code(std::errc::invalid_argument);
      return false;
    }

    logger_.info("Setting timing budget to {} us.", budget_us);

    // get the oscillator frequency (1 byte from register 0x0006)
    uint8_t data[2];
    if (!read_reg(Register::OSCILLATOR_FREQUENCY, data, 2, ec)) {
      return false;
    }
    uint16_t oscillator_frequency = (data[0] << 8) | data[1];

    int macro_period_us = int(2304 * (0x40000000 / oscillator_frequency)) >> 6;

    logger_.debug("Oscillator frequency: {}", oscillator_frequency);
    logger_.debug("Macro period: {} us", macro_period_us);
    logger_.debug("Intermeasurement period: {} ms", intermeasurement_period_ms);

    if (intermeasurement_period_ms == 0) {
      // we're in continuous mode
      budget_us -= 2500;
    } else {
      // we're in autonomous mode
      budget_us -= 4300;
      budget_us /= 2;
    }

    uint16_t ms_byte = 0;
    budget_us <<= 12;
    int tmp = macro_period_us * 16;
    int ls_byte = int(((budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1);
    while ((ls_byte & 0xFFFFFF00) > 0) {
      ls_byte >>= 1;
      ms_byte++;
    }
    ms_byte = (ms_byte << 8) + (ls_byte & 0xFF);
    uint8_t ms_byte_array[2];
    ms_byte_array[0] = ms_byte >> 8;
    ms_byte_array[1] = ms_byte & 0x00FF;
    logger_.debug("ms_byte: {:#04x}, {:#04x}", ms_byte_array[0], ms_byte_array[1]);
    if (!write_reg(Register::RANGE_CONFIG_A, ms_byte_array, 2, ec)) {
      return false;
    }

    ms_byte = 0;
    tmp = macro_period_us * 12;
    ls_byte = int(((budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1);
    while ((ls_byte & 0xFFFFFF00) > 0) {
      ls_byte >>= 1;
      ms_byte++;
    }
    ms_byte = (ms_byte << 8) + (ls_byte & 0xFF);
    ms_byte_array[0] = ms_byte >> 8;
    ms_byte_array[1] = ms_byte & 0x00FF;
    logger_.debug("ms_byte: {:#04x}, {:#04x}", ms_byte_array[0], ms_byte_array[1]);
    if (!write_reg(Register::RANGE_CONFIG_B, ms_byte_array, 2, ec)) {
      return false;
    }

    logger_.debug("Timing budget set to {} us", budget_us);
    return true;
  }

  /// \brief Set the timing budget in milliseconds
  /// \details
  /// The timing budget is the time allowed for one measurement. Valid range
  /// is 10ms to 200ms.
  /// \param budget_ms Timing budget in milliseconds
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  /// \see get_timing_budget_us()
  /// \see set_timing_budget_us()
  bool set_timing_budget_ms(uint32_t budget_ms, std::error_code &ec) {
    return set_timing_budget_us(budget_ms * 1000, ec);
  }

protected:
  static constexpr uint8_t MODEL_ID = 0xEB;
  static constexpr uint8_t MODULE_TYPE = 0xAA;

  static constexpr int MIN_TIMING_BUDGET_US = 10000;  // 10 ms
  static constexpr int MAX_TIMING_BUDGET_US = 200000; // 200 ms

  static constexpr uint8_t INIT_SEQUENCE[] = {
      // value    addr : description
      0x00, // 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch
      0x00, // 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
      0x00, // 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
      0x11, // 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must
            // be 0x1)
      0x02, // 0x31 : bit 1 = interrupt depending on the polarity
      0x00, // 0x32 : not user-modifiable
      0x02, // 0x33 : not user-modifiable
      0x08, // 0x34 : not user-modifiable
      0x00, // 0x35 : not user-modifiable
      0x08, // 0x36 : not user-modifiable
      0x10, // 0x37 : not user-modifiable
      0x01, // 0x38 : not user-modifiable
      0x01, // 0x39 : not user-modifiable
      0x00, // 0x3a : not user-modifiable
      0x00, // 0x3b : not user-modifiable
      0x00, // 0x3c : not user-modifiable
      0x00, // 0x3d : not user-modifiable
      0xFF, // 0x3e : not user-modifiable
      0x00, // 0x3f : not user-modifiable
      0x0F, // 0x40 : not user-modifiable
      0x00, // 0x41 : not user-modifiable
      0x00, // 0x42 : not user-modifiable
      0x00, // 0x43 : not user-modifiable
      0x00, // 0x44 : not user-modifiable
      0x00, // 0x45 : not user-modifiable
      0x20, // 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of
            // window, 3->In window, 0x20-> New sample ready , TBC
      0x0B, // 0x47 : not user-modifiable
      0x00, // 0x48 : not user-modifiable
      0x00, // 0x49 : not user-modifiable
      0x02, // 0x4a : not user-modifiable
      0x14, // 0x4b : not user-modifiable
      0x21, // 0x4c : not user-modifiable
      0x00, // 0x4d : not user-modifiable
      0x00, // 0x4e : not user-modifiable
      0x05, // 0x4f : not user-modifiable
      0x00, // 0x50 : not user-modifiable
      0x00, // 0x51 : not user-modifiable
      0x00, // 0x52 : not user-modifiable
      0x00, // 0x53 : not user-modifiable
      0xC8, // 0x54 : not user-modifiable
      0x00, // 0x55 : not user-modifiable
      0x00, // 0x56 : not user-modifiable
      0x38, // 0x57 : not user-modifiable
      0xFF, // 0x58 : not user-modifiable
      0x01, // 0x59 : not user-modifiable
      0x00, // 0x5a : not user-modifiable
      0x08, // 0x5b : not user-modifiable
      0x00, // 0x5c : not user-modifiable
      0x00, // 0x5d : not user-modifiable
      0x01, // 0x5e : not user-modifiable
      0xCC, // 0x5f : not user-modifiable
      0x07, // 0x60 : not user-modifiable
      0x01, // 0x61 : not user-modifiable
      0xF1, // 0x62 : not user-modifiable
      0x05, // 0x63 : not user-modifiable
      0x00, // 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), default value 90 mm
      0xA0, // 0x65 : Sigma threshold LSB
      0x00, // 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB)
      0x80, // 0x67 : Min count Rate LSB
      0x08, // 0x68 : not user-modifiable
      0x38, // 0x69 : not user-modifiable
      0x00, // 0x6a : not user-modifiable
      0x00, // 0x6b : not user-modifiable
      0x00, // 0x6c : Intermeasurement period MSB, 32 bits register
      0x00, // 0x6d : Intermeasurement period
      0x0F, // 0x6e : Intermeasurement period
      0x89, // 0x6f : Intermeasurement period LSB
      0x00, // 0x70 : not user-modifiable
      0x00, // 0x71 : not user-modifiable
      0x00, // 0x72 : distance threshold high MSB (in mm, MSB+LSB)
      0x00, // 0x73 : distance threshold high LSB
      0x00, // 0x74 : distance threshold low MSB ( in mm, MSB+LSB)
      0x00, // 0x75 : distance threshold low LSB
      0x00, // 0x76 : not user-modifiable
      0x01, // 0x77 : not user-modifiable
      0x07, // 0x78 : not user-modifiable
      0x05, // 0x79 : not user-modifiable
      0x06, // 0x7a : not user-modifiable
      0x06, // 0x7b : not user-modifiable
      0x00, // 0x7c : not user-modifiable
      0x00, // 0x7d : not user-modifiable
      0x02, // 0x7e : not user-modifiable
      0xC7, // 0x7f : not user-modifiable
      0xFF, // 0x80 : not user-modifiable
      0x9B, // 0x81 : not user-modifiable
      0x00, // 0x82 : not user-modifiable
      0x00, // 0x83 : not user-modifiable
      0x00, // 0x84 : not user-modifiable
      0x01, // 0x85 : not user-modifiable
      0x00, // 0x86 : clear interrupt, 0x01=clear
      0x00  // 0x87 : ranging, 0x00=stop, 0x40=start
  };

  enum class Register : uint16_t {
    RESET = 0x0000,                ///< Device reset
    I2C_ADDRESS = 0x0001,          ///< I2C bus device address
    OSCILLATOR_FREQUENCY = 0x0006, ///< Oscillator frequency (two bytes, high then low)
    VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = 0x0008,       ///< Range timeout
    SYSTEM_SEQUENCE_CONFIG = 0x000B,                     ///< Sequence config
    XTALK_PLANE_OFFSET_KCPS = 0x0016,                    ///< XTalk calibration plane offset
    XTALK_X_PLANE_GRADIENT_KCPS = 0x0018,                ///< XTalk calibration plane gradient
    XTALK_Y_PLANE_GRADIENT_KCPS = 0x001A,                ///< XTalk calibration plane gradient
    RANGE_OFFSET_MM = 0x001E,                            ///< Offset correction in range distance
    INNER_OFFSET_MM = 0x0020,                            ///< Offset correction in range distance
    OUTER_OFFSET_MM = 0x0022,                            ///< Offset correction in range distance
    I2C_FAST_MODE_PLUS = 0x002D,                         ///< Enable fast mode plus
    GPIO_HV_MUX_CTRL = 0x0030,                           ///< GPIO selection
    GPIO_TIO_HV_STATUS = 0x0031,                         ///< GPIO selection
    SYSTEM_INTERRUPT = 0x0046,                           ///< Interrupt configuration
    RANGE_CONFIG_A = 0x005E,                             ///< Timing budget
    RANGE_CONFIG_B = 0x0061,                             ///< Intermeasurement period
    RANGE_CONFIG_SIGMA_THRESH = 0x0064,                  ///< Sigma threshold
    RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS = 0x0066, ///< Min count rate
    INTERMEASUREMENT_MS = 0x006C,                        ///< Intermeasurement period
    THRESH_HIGH = 0x0072,                                ///< Threshold high for ranging
    THRESH_LOW = 0x0074,                                 ///< Threshold low for ranging
    SYSTEM_INTERRUPT_CLEAR = 0x0086,                     ///< Interrupt clear
    SYSTEM_START = 0x0087,                               ///< Start/stop measurement
    RESULT_RANGE_STATUS = 0x0089,                        ///< Range status
    RESULT_SPAD_COUNT = 0x008C,                          ///< SPAD count
    RESULT_SIGNAL_RATE = 0x008E,                         ///< Signal rate
    RESULT_AMBIENT_RATE = 0x0090,                        ///< Ambient rate
    RESULT_SIGMA = 0x0092,                               ///< Sigma estimate
    RESULT_DISTANCE_MM = 0x0096,                         ///< Measured distance
    OSC_CALIBRATE_VAL = 0x00DE,                          ///< Oscillator calibration
    FIRMWARE_SYSTEM_STATUS = 0x00E5,                     ///< System status register
    IDENTIFICATION_MODEL_ID = 0x010F,                    ///< Model ID
  };

  /// \brief Initialize the sensor
  /// \param config Configuration
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  /// \see Config
  bool init(const Config &config, std::error_code &ec) {
    // get the model info
    [[maybe_unused]] auto model_info = get_model_info(ec);
    if (ec) {
      return false;
    }
    // wait for boot
    if (!wait_for_boot(ec)) {
      return false;
    }
    // write init sequence to x002d
    if (!write_reg(Register::I2C_FAST_MODE_PLUS, const_cast<uint8_t *>(INIT_SEQUENCE),
                   sizeof(INIT_SEQUENCE), ec)) {
      return false;
    }
    // start vhv
    if (!start_vhv(ec)) {
      return false;
    }
    // clear interrupt
    if (!clear_interrupt(ec)) {
      return false;
    }
    // stop ranging
    if (!stop_ranging(ec)) {
      return false;
    }
    // write 0x09 to VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND
    if (!write_reg(Register::VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09, ec)) {
      return false;
    }
    // write 0x00 to SYSTEM_SEQUENCE_CONFIG
    if (!write_reg(Register::SYSTEM_SEQUENCE_CONFIG, 0x00, ec)) {
      return false;
    }
    // write 0x0500 to 0x0024 (VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT?)
    uint16_t val = 0x0005;
    if (!write_reg(Register::RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS, (uint8_t *)&val, 2, ec)) {
      return false;
    }
    return true;
  }

  /// \brief Wait for the sensor to boot
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  bool wait_for_boot(std::error_code &ec) {
    // wait until the FIRMWARE_SYSTEM_STATUS register is 0x03, sleeping 1ms
    // between checks
    logger_.debug("Waiting for boot");
    uint8_t status;
    do {
      if (!read_reg(Register::FIRMWARE_SYSTEM_STATUS, &status, 1, ec)) {
        return false;
      }
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(1ms);
    } while (status != 0x03);
    logger_.debug("Sensor booted");
    return true;
  }

  /// \brief Start the VHV
  /// \param ec Error code if unsuccessful
  /// \details
  /// Start the VHV (voltage high) to power the sensor. This is required before
  /// ranging can begin.
  /// \see start_ranging()
  /// \see stop_ranging()
  /// \see is_data_ready()
  bool start_vhv(std::error_code &ec) {
    logger_.info("Starting VHV");
    // start ranging
    if (!start_ranging(ec)) {
      return false;
    }
    logger_.debug("waiting for data to be ready");
    // wait for data to be ready
    while (!is_data_ready(ec)) {
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(1ms);
    }
    // clear the interrupt
    clear_interrupt(ec);
    return !ec;
  }

  /// \brief Get the interrupt polarity
  /// \return True if active high, false if active low
  bool get_interrupt_polarity() const { return interrupt_polarity_; }

  /// \brief Get the interrupt polarity
  /// \param ec Error code if unsuccessful
  /// \return True if active high, false if active low
  bool read_interrupt_polarity(std::error_code &ec) {
    // read the GPIO_HV_MUX_CTRL register and return true if the fourth bit is
    // set
    uint8_t val;
    if (!read_reg(Register::GPIO_HV_MUX_CTRL, &val, 1, ec)) {
      return false;
    }
    val = (val >> 4) & 0x01;
    // 0 = active high, 1 = active low
    bool polarity = val != 1;
    logger_.debug("Interrupt polarity: {}", polarity);
    // store the interrupt polarity
    interrupt_polarity_ = polarity;
    return polarity;
  }

  /// \brief Write to a register
  /// \param reg Register address
  /// \param val Value to write
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  /// \see read_reg
  /// \see write_reg
  bool write_reg(Register reg, uint8_t val, std::error_code &ec) {
    return write_reg(reg, &val, 1, ec);
  }

  /// \brief Write to a register
  /// \param reg Register address
  /// \param data Data to write
  /// \param len Number of bytes to write
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  /// \see read_reg
  /// \see write_reg
  bool write_reg(Register reg, uint8_t *data, size_t len, std::error_code &ec) {
    write_many_to_register(static_cast<vl53l_register_t>(reg), data, len, ec);
    return !ec;
  }

  /// \brief Read from a register
  /// \param reg Register address
  /// \param val Value to read
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  /// \see write_reg
  /// \see read_reg
  bool read_reg(Register reg, uint8_t *val, std::error_code &ec) {
    return read_reg(reg, val, 1, ec);
  }

  /// \brief Read from a register
  /// \param reg Register address
  /// \param val Value to read
  /// \param len Number of bytes to read
  /// \param ec Error code if unsuccessful
  /// \return True if successful
  /// \see write_reg
  /// \see read_reg
  bool read_reg(Register reg, uint8_t *val, size_t len, std::error_code &ec) {
    read_many_from_register(static_cast<vl53l_register_t>(reg), val, len, ec);
    return !ec;
  }

  bool ranging_{false};
  bool continuous_{false};
  std::atomic<bool> interrupt_polarity_{false};
  std::atomic<int> inter_measurement_period_ms_{0};
  std::atomic<int> timing_budget_us_{0};
};
} // namespace espp
