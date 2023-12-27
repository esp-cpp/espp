#pragma once

#include <functional>
#include <mutex>

#include "logger.hpp"

namespace espp {
  /**
   * @brief Class to interface with the MAX1704x battery fuel gauge.
   * @details This class is used to interface with the MAX1704x battery fuel
   *          gauge. It is used to get the battery voltage, state of charge, and
   *          charge/discharge rate.
   */
  class Max1704x {
  public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x36; ///< Default address of the MAX1704x.

    /**
     * @brief Function to write bytes to the device.
     * @param dev_addr Address of the device to write to.
     * @param data Pointer to array of bytes to write.
     * @param data_len Number of data bytes to write.
     * @return True if successful, false otherwise.
     */
    typedef std::function<bool(uint8_t dev_addr, uint8_t *data, size_t data_len)> write_fn;

    /**
     * @brief Function to read bytes from the device.
     * @param dev_addr Address of the device to write to.
     * @param data Pointer to array of bytes to read into.
     * @param data_len Number of data bytes to read.
     * @return True if successful, false otherwise.
     */
    typedef std::function<bool(uint8_t dev_addr, uint8_t *data, size_t data_len)>
    read_fn;

    enum class AlertStatus {
      SOC_CHANGE = 0x20, ///< Alert for state of charge change
      SOC_LOW = 0x10, ///< Alert for state of charge low
      VOLTAGE_RESET = 0x08, ///< Alert for voltage reset dip
      VOLTAGE_LOW = 0x04, ///< Alert for voltage low
      VOLTAGE_HIGH = 0x02, ///< Alert for voltage high
    };

    /**
     * @brief Configuration for the MAX1704x.
     */
    struct Config {
      uint8_t device_address{DEFAULT_ADDRESS}; ///< Address of the MAX1704x.
      write_fn write; //< Function to write bytes to the device.
      read_fn read; //< Function to read bytes from the device.
      bool auto_init{true}; ///< Whether to automatically initialize the MAX1704x.
      Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log level for the MAX1704x.
    };

    /**
     * @brief Construct a new Max1704x object.
     * @param config Configuration for the MAX1704x.
     */
    explicit Max1704x(const Config& config)
      : address_(config.device_address),
        write_(config.write),
        read_(config.read),
        logger_({.tag="Max1704x", .level=config.log_level}) {
      if (config.auto_init) {
        std::error_code ec;
        initalize(ec);
        if (ec) {
          logger_.error("Failed to initialize MAX1704x: {}", ec.message());
        }
      }
    }

    /**
     * @brief Delete the Max1704x object.
     */
    ~Max1704x();

    /**
     * @brief Initialize the MAX1704x.
     */
    void initalize(std::error_code& ec) {
      std::scoped_lock lock(mutex_);
      // Get the IC version
      version_ = get_version(ec);
      if (ec) {
        return;
      }
      // Get the chip ID
      chip_id_ = get_chip_id(ec);
      if (ec) {
        return;
      }
      logger_.info("MAX1704x version: 0x{:04X}, chip ID: 0x{:02X}", version_, chip_id_);
      ec.clear();
    }

    /**
     * @brief Get the IC version.
     * @param ec Error code set if an error occurs.
     * @return The IC version.
     */
    uint16_t get_version(std::error_code& ec) {
      std::scoped_lock lock(mutex_);
      uint16_t data = read_register(Register::VERSION, ec);
      if (ec) {
        return 0;
      }
      ec.clear();
      return data;
    }

    /**
     * @brief Get the Chip ID.
     * @param ec Error code set if an error occurs.
     * @return The chip ID.
     */
    uint8_t get_chip_id(std::error_code& ec) {
      std::scoped_lock lock(mutex_);
      uint16_t data = read_register(Register::CHIPID, ec);
      if (ec) {
        return 0;
      }
      ec.clear();
      return data;
    }

    /**
     * @brief Get the battery voltage.
     * @param ec Error code set if an error occurs.
     * @return The battery voltage in V.
     */
    float get_battery_voltage(std::error_code& ec) {
      std::scoped_lock lock(mutex_);
      uint16_t data = read_register(Register::VCELL, ec);
      if (ec) {
        return 0;
      }
      ec.clear();
      return (float)data * CELL_VOLTAGE_FACTOR;
    }

    /**
     * @brief Get the battery state of charge.
     * @details This is the percentage of battery charge remaining.
     * @param ec Error code set if an error occurs.
     * @return The battery state of charge in %.
     */
    float get_battery_percentage(std::error_code& ec) {
      std::scoped_lock lock(mutex_);
      uint16_t data = read_register(Register::SOC, ec);
      if (ec) {
        return 0;
      }
      ec.clear();
      return (float)data * CELL_SOC_FACTOR;
    }

    /**
     * @brief Get the battery charge or discharge rate.
     * @details This is the rate at which the battery is charging or
     *          discharging. A positive value indicates charging. A negative
     *          value indicates discharging. Units are in % per hour.
     * @param ec Error code set if an error occurs.
     * @return The battery charge or discharge rate in %/hr.
     */
    float get_battery_charge_rate(std::error_code& ec) {
      std::scoped_lock lock(mutex_);
      uint16_t data = read_register(Register::CRATE, ec);
      if (ec) {
        return 0;
      }
      ec.clear();
      return (float)data * CELL_CRATE_FACTOR;
    }

    /**
     * @brief Get the alert status.
     * @details This is the current alert status of the battery.
     * @param ec Error code set if an error occurs.
     * @return The battery alert status as an AlertStatus.
     */
    AlertStatus get_alert_status(std::error_code& ec) {
      std::scoped_lock lock(mutex_);
      uint16_t data = read_register(Register::STATUS, ec);
      if (ec) {
        return AlertStatus::SOC_CHANGE;
      }
      ec.clear();
      return (AlertStatus)(data & 0x7F);
    }

    /**
     * @brief Clear the alert status.
     * @details This clears the alert status of the battery.
     * @param flags_to_clear The flags to clear.
     * @param ec Error code set if an error occurs.
     */
    void clear_alert_status(uint8_t flags_to_clear, std::error_code& ec) {
      std::scoped_lock lock(mutex_);
      uint8_t data = read_register(Register::STATUS, ec);
      if (ec) {
        return;
      }
      data &= ~flags_to_clear;
      write_register(Register::STATUS, data, ec);
    }

  protected:
    static constexpr float CELL_VOLTAGE_FACTOR = 78.125f / 1000000.0f; ///< Factor to convert cell voltage to V. 78.125uV per bit.
    static constexpr float CELL_SOC_FACTOR = 1.0f / 256.0f; ///< Factor to convert cell state of charge to %.
    static constexpr float CELL_CRATE_FACTOR = 0.208f; ///< Factor to convert cell charge rate to %/hr.

    enum class Register {
      VCELL = 0x02,   ///< Register that holds cell voltage
      SOC = 0x04,     ///< Register that holds cell state of charge
      MODE = 0x06,    ///< Register that manages mode. Default is 0x0000
      VERSION = 0x08, ///< Register that has IC version
      HIBRT = 0x0A,   ///< Register that manages hibernation. Default is 0x8030
      CONFIG = 0x0C,  ///< Register that manages configuration. Default is 0x971C
      VALRT = 0x14,   ///< Register that holds voltage alert values
      CRATE = 0x16,   ///< Register that holds cell charge rate
      VRESET = 0x18,  ///< Register that holds reset voltage setting
      CHIPID = 0x19,  ///< Register that holds semi-unique chip ID
      STATUS = 0x1A,  ///< Register that holds current alert/status
      CMD = 0xFE      ///< Register that can be written for special commands
    };

    enum class Alert {
      SOC_CHANGE = 0x20, ///< Alert for state of charge change
      SOC_LOW = 0x10, ///< Alert for state of charge low
      VOLTAGE_RESET = 0x08, ///< Alert for voltage reset dip
      VOLTAGE_LOW = 0x04, ///< Alert for voltage low
      VOLTAGE_HIGH = 0x02, ///< Alert for voltage high
    };

    void write_register(const Register reg, const uint8_t data, std::error_code& ec) {
      std::scoped_lock lock(mutex_);
      uint8_t buf[2];
      buf[0] = (uint8_t)reg;
      buf[2] = (uint8_t)data;
      if (!write_(address_, buf, sizeof(buf))) {
        logger_.error("Failed to write register 0x{:02X}", (uint8_t)reg);
        ec = std::make_error_code(std::errc::io_error);
        return;
      }
      ec.clear();
    }

    uint16_t read_register(const Register reg, std::error_code& ec) {
      std::scoped_lock lock(mutex_);
      uint8_t data[2];
      // write the address
      if (!write_(address_, (uint8_t *)&reg, 1)) {
        ec = std::make_error_code(std::errc::io_error);
        logger_.error("Failed to write register 0x{:02X}", (uint8_t)reg);
        return 0;
      }
      // and read the data
      if (!read_(address_, data, 2)) {
        ec = std::make_error_code(std::errc::io_error);
        logger_.error("Failed to read register 0x{:02X}", (uint8_t)reg);
        return 0;
      }
      ec.clear();
      return (data[0] << 8) | data[1];
    }

    uint16_t version_{0}; ///< IC version
    uint8_t chip_id_{0}; ///< Chip ID

    uint8_t address_;
    write_fn write_;
    read_fn read_;
    std::recursive_mutex mutex_;
    Logger logger_;
  };
} // namespace espp
