#pragma once

#include "base_component.hpp"
#include "base_peripheral.hpp"
#include <concepts>
#include <cstdint>
#include <mutex>
#include <string_view>
#include <system_error>
#include <type_traits>

namespace espp {
/// @brief Class for the ADXL345 I2C 3-axis accelerometer
///
/// The ADXL345 is a 3-axis accelerometer with 10-bit to 13-bit resolution
/// measurement at up to ±16g. Digital output data is formatted as 16-bit twos
/// complement and is accessible through either a SPI (3- or 4-wire) or I2C
/// digital interface.
///
/// The ADXL345 can support SPI up to 5 MHz and I2C up to 400 kHz. It has a
/// programmable interrupt system that can be used to signal events such as
/// data ready, free-fall, activity, inactivity, and tap detection.
///
/// The datasheet can be found here:
/// https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf
///
/// \section adxl345_ex_1 ADXL345 Example
/// \snippet adxl345_example.cpp adxl345 example
class Adxl345 : public BasePeripheral<> {
public:
  static constexpr std::uint8_t DEFAULT_ADDRESS = 0x53; ///< Default I2C address of the ADXL345

  /// @brief Data structure for accelerometer readings
  struct Data {
    float x; ///< X-axis acceleration in g
    float y; ///< Y-axis acceleration in g
    float z; ///< Z-axis acceleration in g
  };

  /// @brief Range of the accelerometer
  enum Range {
    RANGE_2G = 0,  ///< ±2g
    RANGE_4G = 1,  ///< ±4g
    RANGE_8G = 2,  ///< ±8g
    RANGE_16G = 3, ///< ±16g
  };

  /// @brief Output data rate
  enum DataRate {
    RATE_0_10_HZ = 0,  ///< 0.10 Hz
    RATE_0_20_HZ = 1,  ///< 0.20 Hz
    RATE_0_39_HZ = 2,  ///< 0.39 Hz
    RATE_0_78_HZ = 3,  ///< 0.78 Hz
    RATE_1_56_HZ = 4,  ///< 1.56 Hz
    RATE_3_13_HZ = 5,  ///< 3.13 Hz
    RATE_6_25_HZ = 6,  ///< 6.25 Hz
    RATE_12_5_HZ = 7,  ///< 12.5 Hz
    RATE_25_HZ = 8,    ///< 25 Hz
    RATE_50_HZ = 9,    ///< 50 Hz
    RATE_100_HZ = 10,  ///< 100 Hz
    RATE_200_HZ = 11,  ///< 200 Hz
    RATE_400_HZ = 12,  ///< 400 Hz
    RATE_800_HZ = 13,  ///< 800 Hz
    RATE_1600_HZ = 14, ///< 1600 Hz
    RATE_3200_HZ = 15, ///< 3200 Hz
  };

  /// @brief Interrupt Pin
  enum InterruptPin {
    INT1 = 0x00, ///< INT1 pin
    INT2 = 0x01, ///< INT2 pin
  };

  /// @brief Interrupt type
  enum InterruptType {
    OVERRUN = 0,    ///< FIFO overrun interrupt
    WATERMARK = 1,  ///< FIFO watermark interrupt
    FREE_FALL = 2,  ///< Free fall interrupt
    INACTIVITY = 3, ///< Inactivity interrupt
    ACTIVITY = 4,   ///< Activity interrupt
    DOUBLE_TAP = 5, ///< Double tap interrupt
    SINGLE_TAP = 6, ///< Single tap interrupt
    DATA_READY = 7, ///< Data ready interrupt
  };

  /// @brief FIFO Mode
  enum class FifoMode {
    BYPASS = 0,  ///< Bypass mode
    FIFO = 1,    ///< FIFO mode
    STREAM = 2,  ///< Stream mode
    TRIGGER = 3, ///< Trigger mode
  };

  /// @brief Configuration for the ADXL345
  struct Config {
    std::uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address of the device
    Range range = RANGE_2G;                        ///< Range of the accelerometer
    DataRate data_rate = RATE_100_HZ;              ///< Output data rate
    bool auto_init = true; ///< Automatically initialize the device on construction
    BasePeripheral<std::uint8_t, true>::write_fn write; ///< Function to write to the device
    BasePeripheral<std::uint8_t, true>::read_fn read;   ///< Function to read from the device
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Verbosity for the logger
  };

  /// @brief Constructor
  /// @param config Configuration for the ADXL345
  explicit Adxl345(const Config &config)
      : BasePeripheral<std::uint8_t, true>(
            {.address = config.device_address, .write = config.write, .read = config.read},
            "Adxl345", config.log_level)
      , config_(config) {
    if (config.auto_init) {
      std::error_code ec;
      initialize(ec);
    }
  }

  /// @brief Initialize the ADXL345
  /// @param ec Error code to set if an error occurs
  /// @return True if initialization was successful, false otherwise
  bool initialize(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // Check device ID
    std::uint8_t device_id = read_u8_from_register(Register::DEVID, ec);
    if (ec) {
      return false;
    }
    if (device_id != DEVICE_ID) {
      logger_.error("Invalid device ID: 0x{:02x}, expected 0x{:02x}", device_id, DEVICE_ID);
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }

    // disable all interrupts
    if (!disable_all_interrupts(ec)) {
      return false;
    }

    // Set range
    if (!set_range(config_.range, ec)) {
      return false;
    }

    // Set data rate
    if (!set_data_rate(config_.data_rate, ec)) {
      return false;
    }

    // Enable measurement mode
    if (!set_measurement_mode(true, ec)) {
      return false;
    }

    return true;
  }

  /// @brief Set the range of the accelerometer
  /// @param range New range
  /// @param ec Error code to set if an error occurs
  /// @return True if setting range was successful, false otherwise
  bool set_range(Range range, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Setting accelerometer range to {}", range);

    // Read current data format
    std::uint8_t data_format = read_u8_from_register(Register::DATA_FORMAT, ec);
    if (ec) {
      logger_.error("Failed to read DATA_FORMAT register: {}", ec.message());
      return false;
    }

    // Clear range bits and set new range
    data_format &= ~0x03;
    data_format |= static_cast<std::uint8_t>(range);

    // Write new data format
    write_u8_to_register(Register::DATA_FORMAT, data_format, ec);
    if (ec) {
      logger_.error("Failed to write DATA_FORMAT register: {}", ec.message());
      return false;
    }

    return true;
  }

  /// @brief Set the output data rate
  /// @param rate New data rate
  /// @param ec Error code to set if an error occurs
  /// @return True if setting data rate was successful, false otherwise
  bool set_data_rate(DataRate rate, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Setting output data rate to {}", rate);

    // Read current bandwidth rate
    std::uint8_t bw_rate = read_u8_from_register(Register::BW_RATE, ec);
    if (ec) {
      logger_.error("Failed to read BW_RATE register: {}", ec.message());
      return false;
    }

    // Clear rate bits and set new rate
    bw_rate &= ~0x0F;
    bw_rate |= static_cast<std::uint8_t>(rate);

    // Write new bandwidth rate
    write_u8_to_register(Register::BW_RATE, bw_rate, ec);
    if (ec) {
      logger_.error("Failed to write BW_RATE register: {}", ec.message());
      return false;
    }

    return true;
  }

  /// @brief Enable/Disable low power mode
  /// @param enable Whether to enable or disable low power mode
  /// @param ec Error code to set if an error occurs
  /// @return True if setting low power mode was successful, false otherwise
  bool set_low_power(bool enable, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Setting low power mode to {}", enable ? "enabled" : "disabled");

    // Read current BW_RATE register
    std::uint8_t bw_rate = read_u8_from_register(Register::BW_RATE, ec);
    if (ec) {
      logger_.error("Failed to read BW_RATE register: {}", ec.message());
      return false;
    }

    static constexpr int low_power_bit = 4;

    // Set or clear the low power bit
    if (enable) {
      bw_rate |= (1 << low_power_bit); // Set
    } else {
      bw_rate &= ~(1 << low_power_bit); // Clear
    }

    // Write new BW_RATE value
    write_u8_to_register(Register::BW_RATE, bw_rate, ec);
    if (ec) {
      logger_.error("Failed to write BW_RATE register: {}", ec.message());
      return false;
    }

    return true;
  }

  /// @brief Enable/Disable the auto-sleep mode
  /// @param enable Whether to enable or disable auto-sleep
  /// @param ec Error code to set if an error occurs
  /// @return True if setting auto-sleep was successful, false otherwise
  bool set_auto_sleep(bool enable, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Setting auto-sleep mode to {}", enable ? "enabled" : "disabled");

    // Read current power control register
    std::uint8_t power_ctl = read_u8_from_register(Register::POWER_CTL, ec);
    if (ec) {
      logger_.error("Failed to read POWER_CTL register: {}", ec.message());
      return false;
    }

    static constexpr int auto_sleep_bit = 4;

    // Set or clear the auto-sleep bit
    if (enable) {
      power_ctl |= (1 << auto_sleep_bit); // Set
    } else {
      power_ctl &= ~(1 << auto_sleep_bit); // Clear
    }

    // Write new power control value
    write_u8_to_register(Register::POWER_CTL, power_ctl, ec);
    if (ec) {
      logger_.error("Failed to write POWER_CTL register: {}", ec.message());
      return false;
    }

    return true;
  }

  /// @brief Enable/Disable the measurement mode
  /// @param enable Whether to enable or disable measurement mode
  /// @param ec Error code to set if an error occurs
  /// @return True if setting measurement mode was successful, false otherwise
  bool set_measurement_mode(bool enable, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Setting measurement mode to {}", enable ? "enabled" : "disabled");

    // Read current power control register
    std::uint8_t power_ctl = read_u8_from_register(Register::POWER_CTL, ec);
    if (ec) {
      return false;
    }

    static constexpr int measure_bit = 3;

    // Set or clear the measure bit
    if (enable) {
      power_ctl |= (1 << measure_bit); // Set
    } else {
      power_ctl &= ~(1 << measure_bit); // Clear
    }

    // Write new power control value
    write_u8_to_register(Register::POWER_CTL, power_ctl, ec);
    if (ec) {
      return false;
    }

    return true;
  }

  /// @brief Enable/Disable the Sleep mode
  /// @param enable Whether to enable or disable sleep mode
  /// @param ec Error code to set if an error occurs
  /// @return True if setting sleep mode was successful, false otherwise
  bool set_sleep_mode(bool enable, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Setting sleep mode to {}", enable ? "enabled" : "disabled");

    // Read current power control register
    std::uint8_t power_ctl = read_u8_from_register(Register::POWER_CTL, ec);
    if (ec) {
      logger_.error("Failed to read POWER_CTL register: {}", ec.message());
      return false;
    }

    static constexpr int sleep_bit = 2;

    // Set or clear the sleep bit
    if (enable) {
      power_ctl |= (1 << sleep_bit); // Set
    } else {
      power_ctl &= ~(1 << sleep_bit); // Clear
    }

    // Write new power control value
    write_u8_to_register(Register::POWER_CTL, power_ctl, ec);
    if (ec) {
      logger_.error("Failed to write POWER_CTL register: {}", ec.message());
      return false;
    }

    return true;
  }

  /// @brief Set the FIFO mode
  /// @param mode New FIFO mode
  /// @param ec Error code to set if an error occurs
  /// @return True if setting FIFO mode was successful, false otherwise
  bool set_fifo_mode(FifoMode mode, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Setting FIFO mode to {}", mode);

    // Read current FIFO control register
    std::uint8_t fifo_ctl = read_u8_from_register(Register::FIFO_CTL, ec);
    if (ec) {
      logger_.error("Failed to read FIFO_CTL register: {}", ec.message());
      return false;
    }

    // fifo mode bits are bits 6 and 7 of FIFO_CTL register
    static constexpr int fifo_mode_bits = 0x03; // Bits 0 and 1
    static constexpr int fifo_mode_shift = 6;   // Bits 6 and 7
    static constexpr int fifo_mode_bitmask = fifo_mode_bits << fifo_mode_shift;

    int new_fifo_setting = static_cast<int>(mode) << fifo_mode_shift;

    // Clear the current FIFO mode bits
    fifo_ctl &= ~fifo_mode_bitmask;
    // Set the new FIFO mode bits
    fifo_ctl |= new_fifo_setting;

    // Write new FIFO control value
    write_u8_to_register(Register::FIFO_CTL, fifo_ctl, ec);
    if (ec) {
      logger_.error("Failed to write FIFO_CTL register: {}", ec.message());
      return false;
    }

    return true;
  }

  /// @brief Get the current FIFO mode
  /// @param ec Error code to set if an error occurs
  /// @return Current FIFO mode
  FifoMode get_fifo_mode(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("Reading FIFO mode");
    std::uint8_t fifo_ctl = read_u8_from_register(Register::FIFO_CTL, ec);
    if (ec) {
      logger_.error("Failed to read FIFO_CTL register: {}", ec.message());
      return FifoMode::BYPASS; // Return a default value in case of error
    }

    static constexpr int fifo_mode_bits = 0x03; // Bits 0 and 1
    static constexpr int fifo_mode_shift = 6;   // Bits 6 and 7
    static constexpr int fifo_mode_bitmask = fifo_mode_bits << fifo_mode_shift;

    FifoMode mode = static_cast<FifoMode>((fifo_ctl & fifo_mode_bitmask) >> fifo_mode_shift);
    logger_.info("Current FIFO mode: {}", mode);
    return mode;
  }

  /// @brief Set the FIFO Num Samples
  /// @param num_samples Number of samples to store in FIFO
  /// @param ec Error code to set if an error occurs
  /// @return True if setting FIFO num samples was successful, false otherwise
  /// @note Num Samples is meaningless for FifoMode::BYPASS.
  /// @note The FIFO can store up to 32 samples.
  /// @note Num Samples specifies how many FIFO entries are needed to trigger a
  ///       InterruptType::WATERMARK interrupt in FifoMode::FIFO, and
  ///       FifoMode::STREAM.
  /// @note Num Samples specifies how many FIFO entries are retained in the FIFO
  ///       before an InterruptType::TRIGGER interrupt is triggered.
  bool set_fifo_num_samples(std::uint8_t num_samples, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Setting FIFO number of samples to {}", num_samples);

    // Validate num_samples
    if (num_samples > 31) {
      logger_.error("Invalid number of samples: {}. Must be between 0 and 32.", num_samples);
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }

    // Read current FIFO control register
    std::uint8_t fifo_ctl = read_u8_from_register(Register::FIFO_CTL, ec);
    if (ec) {
      logger_.error("Failed to read FIFO_CTL register: {}", ec.message());
      return false;
    }

    // Set the number of samples bits (bits 0-4)
    fifo_ctl &= ~0x1F;                // Clear bits 0-4
    fifo_ctl |= (num_samples & 0x1F); // Set new value

    // Write new FIFO control value
    write_u8_to_register(Register::FIFO_CTL, fifo_ctl, ec);
    if (ec) {
      logger_.error("Failed to write FIFO_CTL register: {}", ec.message());
      return false;
    }

    return true;
  }

  /// @brief Get the number of samples available in the FIFO
  /// @param ec Error code to set if an error occurs
  /// @return Number of samples available in the FIFO (0-32)
  std::uint8_t get_fifo_num_samples_available(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Reading number of samples available in FIFO");

    // Read FIFO status register
    std::uint8_t fifo_status = read_u8_from_register(Register::FIFO_STATUS, ec);
    if (ec) {
      logger_.error("Failed to read FIFO_STATUS register: {}", ec.message());
      return 0; // Return 0 on error
    }

    // Number of samples is bits 0-5
    return fifo_status & 0x3F; // Mask to get the number of samples
  }

  /// @brief Get the current FIFO status
  /// @param ec Error code to set if an error occurs
  /// @return Current FIFO status as a byte
  /// @note The FIFO status byte contains the following bits:
  ///        - Bit 7: FIFO trigger
  ///        - Bit 6: unused (always 0)
  ///        - Bits 5-0: Number of entries in FIFO (0-32)
  std::uint8_t get_fifo_status(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("Reading FIFO status");
    return read_u8_from_register(Register::FIFO_STATUS, ec);
  }

  /// @brief Read the accelerometer data
  /// @param ec Error code to set if an error occurs
  /// @return Accelerometer data in g
  Data read(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Reading accelerometer data");

    // Read 6 bytes of data (2 bytes per axis)
    std::uint8_t data[6];
    read_many_from_register(Register::DATAX0, data, 6, ec);
    if (ec) {
      logger_.error("Failed to read accelerometer data: {}", ec.message());
      return {0.0f, 0.0f, 0.0f};
    }

    // Convert to 16-bit values
    std::int16_t x = (data[1] << 8) | data[0];
    std::int16_t y = (data[3] << 8) | data[2];
    std::int16_t z = (data[5] << 8) | data[4];

    // Convert to g
    return {
        static_cast<float>(x) / sensitivity_,
        static_cast<float>(y) / sensitivity_,
        static_cast<float>(z) / sensitivity_,
    };
  }

  /// @brief Read all accelerometer data from the FIFO
  /// @param ec Error code to set if an error occurs
  /// @return Vector of accelerometer data in g
  /// @note This function reads all available data from the FIFO.
  ///       The data is returned as a vector of Data structures.
  /// @note If the FIFO is empty, an empty vector is returned.
  std::vector<Data> read_all(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Reading all accelerometer data from FIFO");

    std::vector<Data> data_vector;

    // Get number of samples available in FIFO
    std::uint8_t num_samples = get_fifo_num_samples_available(ec);
    if (ec) {
      logger_.error("Failed to get number of samples available in FIFO: {}", ec.message());
      return data_vector; // Return empty vector on error
    }

    if (num_samples == 0) {
      logger_.info("No data available in FIFO");
      return data_vector; // Return empty vector if no data
    }

    logger_.info("Number of samples available in FIFO: {}", num_samples);

    // Read all samples from FIFO
    for (std::uint8_t i = 0; i < num_samples; ++i) {
      auto data = read(ec);
      if (ec) {
        logger_.error("Failed to read sample {} from FIFO: {}", i, ec.message());
        return data_vector; // Return existing data if error occurs
      }
      data_vector.emplace_back(std::move(data));
    }

    return data_vector;
  }

  /// @brief Set the interrupt polarity
  /// @param active_high Whether the interrupt should be active high (true) or
  ///        active low (false)
  /// @param ec Error code to set if an error occurs
  /// @return True if setting interrupt polarity was successful, false otherwise
  bool set_interrupt_polarity(bool active_high, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Setting interrupt polarity to {}", active_high ? "active high" : "active low");

    // Read current interrupt enable register
    std::uint8_t data_format = read_u8_from_register(Register::DATA_FORMAT, ec);
    if (ec) {
      logger_.error("Failed to read DATA_FORMAT register: {}", ec.message());
      return false;
    }

    static constexpr int polarity_bit = 5;

    // NOTE: value of 0 means active high, value of 1 means active low.
    // Therefore we have to invert the logic.
    bool active_low = !active_high;

    // Set or clear the polarity bit
    if (active_low) {
      data_format |= (1 << polarity_bit); // Set to active low
    } else {
      data_format &= ~(1 << polarity_bit); // Set to active high
    }

    // Write new interrupt enable value
    write_u8_to_register(Register::DATA_FORMAT, data_format, ec);
    if (ec) {
      logger_.error("Failed to write DATA_FORMAT register: {}", ec.message());
      return false;
    }

    return true;
  }

  /// @brief Disable all interrupts
  /// @param ec Error code to set if an error occurs
  /// @return True if disabling all interrupts was successful, false otherwise
  bool disable_all_interrupts(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Disabling all interrupts");

    // Write new interrupt enable value (all bits cleared)
    write_u8_to_register(Register::INT_ENABLE, 0x00, ec);
    if (ec) {
      logger_.error("Failed to write INT_ENABLE register: {}", ec.message());
      return false;
    }

    return true;
  }

  /// @brief Configure an interrupt
  /// @param type Type of interrupt to configure
  /// @param enable Whether to enable or disable the interrupt
  /// @param ec Error code to set if an error occurs
  /// @return True if configuring interrupt was successful, false otherwise
  bool configure_interrupt(InterruptType type, bool enable, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Configuring interrupt {} to {}", type, enable ? "enabled" : "disabled");

    // Read current interrupt enable register
    std::uint8_t int_enable = read_u8_from_register(Register::INT_ENABLE, ec);
    if (ec) {
      logger_.error("Failed to read INT_ENABLE register: {}", ec.message());
      return false;
    }

    // Set or clear the appropriate bit
    if (enable) {
      int_enable |= (1 << static_cast<std::uint8_t>(type));
    } else {
      int_enable &= ~(1 << static_cast<std::uint8_t>(type));
    }

    // Write new interrupt enable value
    write_u8_to_register(Register::INT_ENABLE, int_enable, ec);
    if (ec) {
      logger_.error("Failed to write INT_ENABLE register: {}", ec.message());
      return false;
    }

    return true;
  }

  /// @brief Get the interrupt source
  /// @param ec Error code to set if an error occurs
  /// @return Bitmap of active interrupts
  std::uint8_t get_interrupt_source(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("Reading interrupt source");
    return read_u8_from_register(Register::INT_SOURCE, ec);
  }

  /// @brief Set the interrupt mapping
  /// @param type Type of interrupt to map
  /// @param pin Interrupt pin to map to
  /// @param ec Error code to set if an error occurs
  /// @return True if setting interrupt mapping was successful, false otherwise
  bool set_interrupt_mapping(InterruptType type, InterruptPin pin, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Setting interrupt mapping: pin={}, type={}", pin, type);

    // Read current interrupt map register
    std::uint8_t int_map = read_u8_from_register(Register::INT_MAP, ec);
    if (ec) {
      logger_.error("Failed to read INT_MAP register: {}", ec.message());
      return false;
    }

    // Set the bit for the specified pin
    if (pin == INT1) {
      int_map &= ~(1 << static_cast<std::uint8_t>(type));
    } else {
      int_map |= (1 << static_cast<std::uint8_t>(type));
    }

    // Write new interrupt map value
    write_u8_to_register(Register::INT_MAP, int_map, ec);
    if (ec) {
      logger_.error("Failed to write INT_MAP register: {}", ec.message());
      return false;
    }

    return true;
  }

protected:
  static constexpr float sensitivity_{1.0f / 0.0039f}; ///< Sensitivity in LSB/g

  /// @brief Register addresses
  enum Register {
    DEVID = 0x00,          ///< Device ID
    THRESH_TAP = 0x1D,     ///< Tap threshold
    OFSX = 0x1E,           ///< X-axis offset
    OFSY = 0x1F,           ///< Y-axis offset
    OFSZ = 0x20,           ///< Z-axis offset
    DUR = 0x21,            ///< Tap duration
    LATENT = 0x22,         ///< Tap latency
    WINDOW = 0x23,         ///< Tap window
    THRESH_ACT = 0x24,     ///< Activity threshold
    THRESH_INACT = 0x25,   ///< Inactivity threshold
    TIME_INACT = 0x26,     ///< Inactivity time
    ACT_INACT_CTL = 0x27,  ///< Axis enable control for activity/inactivity detection
    THRESH_FF = 0x28,      ///< Free-fall threshold
    TIME_FF = 0x29,        ///< Free-fall time
    TAP_AXES = 0x2A,       ///< Axis control for tap/double tap
    ACT_TAP_STATUS = 0x2B, ///< Source of tap/double tap
    BW_RATE = 0x2C,        ///< Data rate and power mode control
    POWER_CTL = 0x2D,      ///< Power-saving features control
    INT_ENABLE = 0x2E,     ///< Interrupt enable control
    INT_MAP = 0x2F,        ///< Interrupt mapping control
    INT_SOURCE = 0x30,     ///< Source of interrupts
    DATA_FORMAT = 0x31,    ///< Data format control
    DATAX0 = 0x32,         ///< X-axis data 0
    DATAX1 = 0x33,         ///< X-axis data 1
    DATAY0 = 0x34,         ///< Y-axis data 0
    DATAY1 = 0x35,         ///< Y-axis data 1
    DATAZ0 = 0x36,         ///< Z-axis data 0
    DATAZ1 = 0x37,         ///< Z-axis data 1
    FIFO_CTL = 0x38,       ///< FIFO control
    FIFO_STATUS = 0x39,    ///< FIFO status
  };

  static constexpr std::uint8_t DEVICE_ID = 0xE5; ///< Expected device ID

  Config config_; ///< Configuration
};                // class Adxl345
} // namespace espp

// for libfmt formatting of Adxl345::Data
template <> struct fmt::formatter<espp::Adxl345::Data> : fmt::formatter<float> {
  template <typename FormatContext>
  auto format(const espp::Adxl345::Data &data, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "{{x: {:.2f} g, y: {:.2f} g, z: {:.2f} g}}", data.x, data.y,
                          data.z);
  }
};

// for libfmt formatting of Adxl345::Range
template <> struct fmt::formatter<espp::Adxl345::Range> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const espp::Adxl345::Range &range, FormatContext &ctx) const {
    switch (range) {
    case espp::Adxl345::RANGE_2G:
      return fmt::format_to(ctx.out(), "±2g");
    case espp::Adxl345::RANGE_4G:
      return fmt::format_to(ctx.out(), "±4g");
    case espp::Adxl345::RANGE_8G:
      return fmt::format_to(ctx.out(), "±8g");
    case espp::Adxl345::RANGE_16G:
      return fmt::format_to(ctx.out(), "±16g");
    default:
      return fmt::format_to(ctx.out(), "Unknown Range");
    }
  }
};

// for libfmt formatting of Adxl345::DataRate
template <> struct fmt::formatter<espp::Adxl345::DataRate> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const espp::Adxl345::DataRate &rate, FormatContext &ctx) const {
    switch (rate) {
    case espp::Adxl345::RATE_0_10_HZ:
      return fmt::format_to(ctx.out(), "0.10 Hz");
    case espp::Adxl345::RATE_0_20_HZ:
      return fmt::format_to(ctx.out(), "0.20 Hz");
    case espp::Adxl345::RATE_0_39_HZ:
      return fmt::format_to(ctx.out(), "0.39 Hz");
    case espp::Adxl345::RATE_0_78_HZ:
      return fmt::format_to(ctx.out(), "0.78 Hz");
    case espp::Adxl345::RATE_1_56_HZ:
      return fmt::format_to(ctx.out(), "1.56 Hz");
    case espp::Adxl345::RATE_3_13_HZ:
      return fmt::format_to(ctx.out(), "3.13 Hz");
    case espp::Adxl345::RATE_6_25_HZ:
      return fmt::format_to(ctx.out(), "6.25 Hz");
    case espp::Adxl345::RATE_12_5_HZ:
      return fmt::format_to(ctx.out(), "12.5 Hz");
    case espp::Adxl345::RATE_25_HZ:
      return fmt::format_to(ctx.out(), "25 Hz");
    case espp::Adxl345::RATE_50_HZ:
      return fmt::format_to(ctx.out(), "50 Hz");
    case espp::Adxl345::RATE_100_HZ:
      return fmt::format_to(ctx.out(), "100 Hz");
    case espp::Adxl345::RATE_200_HZ:
      return fmt::format_to(ctx.out(), "200 Hz");
    case espp::Adxl345::RATE_400_HZ:
      return fmt::format_to(ctx.out(), "400 Hz");
    case espp::Adxl345::RATE_800_HZ:
      return fmt::format_to(ctx.out(), "800 Hz");
    case espp::Adxl345::RATE_1600_HZ:
      return fmt::format_to(ctx.out(), "1600 Hz");
    case espp::Adxl345::RATE_3200_HZ:
      return fmt::format_to(ctx.out(), "3200 Hz");
    default:
      return fmt::format_to(ctx.out(), "Unknown Rate");
    }
  }
};

// for libfmt formatting of Adxl345::InterruptPin
template <> struct fmt::formatter<espp::Adxl345::InterruptPin> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const espp::Adxl345::InterruptPin &pin, FormatContext &ctx) const {
    switch (pin) {
    case espp::Adxl345::INT1:
      return fmt::format_to(ctx.out(), "INT1");
    case espp::Adxl345::INT2:
      return fmt::format_to(ctx.out(), "INT2");
    default:
      return fmt::format_to(ctx.out(), "Unknown Interrupt Pin");
    }
  }
};

// for libfmt formatting of Adxl345::InterruptType
template <> struct fmt::formatter<espp::Adxl345::InterruptType> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const espp::Adxl345::InterruptType &type, FormatContext &ctx) const {
    switch (type) {
    case espp::Adxl345::OVERRUN:
      return fmt::format_to(ctx.out(), "OVERRUN");
    case espp::Adxl345::WATERMARK:
      return fmt::format_to(ctx.out(), "WATERMARK");
    case espp::Adxl345::FREE_FALL:
      return fmt::format_to(ctx.out(), "FREE_FALL");
    case espp::Adxl345::INACTIVITY:
      return fmt::format_to(ctx.out(), "INACTIVITY");
    case espp::Adxl345::ACTIVITY:
      return fmt::format_to(ctx.out(), "ACTIVITY");
    case espp::Adxl345::DOUBLE_TAP:
      return fmt::format_to(ctx.out(), "DOUBLE_TAP");
    case espp::Adxl345::SINGLE_TAP:
      return fmt::format_to(ctx.out(), "SINGLE_TAP");
    case espp::Adxl345::DATA_READY:
      return fmt::format_to(ctx.out(), "DATA_READY");
    default:
      return fmt::format_to(ctx.out(), "Unknown Interrupt Type");
    }
  }
};

// for libfmt formatting of Adxl345::FifoMode
template <> struct fmt::formatter<espp::Adxl345::FifoMode> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const espp::Adxl345::FifoMode &mode, FormatContext &ctx) const {
    switch (mode) {
    case espp::Adxl345::FifoMode::BYPASS:
      return fmt::format_to(ctx.out(), "BYPASS");
    case espp::Adxl345::FifoMode::FIFO:
      return fmt::format_to(ctx.out(), "FIFO");
    case espp::Adxl345::FifoMode::STREAM:
      return fmt::format_to(ctx.out(), "STREAM");
    case espp::Adxl345::FifoMode::TRIGGER:
      return fmt::format_to(ctx.out(), "TRIGGER");
    default:
      return fmt::format_to(ctx.out(), "Unknown FIFO Mode");
    }
  }
};
