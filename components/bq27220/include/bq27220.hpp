#pragma once

#include <functional>
#include <mutex>

#include "base_peripheral.hpp"

namespace espp {
/**
 * @brief Class to interface with the BQ27220 battery fuel gauge.
 * @details This class is used to interface with the Texas Instruments BQ27220
 *          I2C battery fuel gauge (gas gauge). It is used to get the battery
 *          voltage, current, state of charge, state of health, temperature,
 *          capacity, and time-to-empty / time-to-full estimates.
 * @note All data values reported by the BQ27220 are 16-bit little-endian.
 * @see https://www.ti.com/lit/gpn/bq27220
 *
 * @section bq27220_ex1 BQ27220 Example
 * @snippet bq27220_example.cpp bq27220 example
 */
class Bq27220 : public BasePeripheral<> {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x55; ///< Default address of the BQ27220.

  /**
   * @brief Configuration for the BQ27220.
   */
  struct Config {
    uint8_t device_address{DEFAULT_ADDRESS}; ///< Address of the BQ27220.
    BasePeripheral::write_fn write;          //< Function to write bytes to the device.
    BasePeripheral::read_fn read;            //< Function to read bytes from the device.
    bool auto_init{true};                    ///< Whether to automatically initialize the BQ27220.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log level for the BQ27220.
  };

  /**
   * @brief Construct a new Bq27220 object.
   * @param config Configuration for the BQ27220.
   */
  explicit Bq27220(const Config &config)
      : BasePeripheral(
            {.address = config.device_address, .write = config.write, .read = config.read},
            "Bq27220", config.log_level) {
    if (config.auto_init) {
      std::error_code ec;
      initialize(ec);
      if (ec) {
        logger_.error("Failed to initialize BQ27220: {}", ec.message());
      }
    }
  }

  /**
   * @brief Initialize the BQ27220.
   * @details Performs a communications sanity check by reading the battery
   *          status register. This does not perform any data-memory or
   *          subcommand access.
   * @param ec Error code set if an error occurs during initialization.
   */
  void initialize(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // Perform a simple comms sanity read of the BatteryStatus register.
    uint16_t status = read_u16_le((uint8_t)Register::BatteryStatus, ec);
    if (ec) {
      return;
    }
    logger_.info("BQ27220 initialized, battery status: 0x{:04X}", status);
    ec.clear();
  }

  /**
   * @brief Get the battery voltage.
   * @param ec Error code set if an error occurs.
   * @return The battery voltage in mV.
   */
  uint16_t get_voltage_mv(std::error_code &ec) {
    return read_u16_le((uint8_t)Register::Voltage, ec);
  }

  /**
   * @brief Get the instantaneous battery current.
   * @details A positive value indicates charging. A negative value indicates
   *          discharging.
   * @param ec Error code set if an error occurs.
   * @return The battery current in mA (signed).
   */
  int16_t get_current_ma(std::error_code &ec) {
    return read_s16_le((uint8_t)Register::Current, ec);
  }

  /**
   * @brief Get the average battery current.
   * @details A positive value indicates charging. A negative value indicates
   *          discharging.
   * @param ec Error code set if an error occurs.
   * @return The average battery current in mA (signed).
   */
  int16_t get_average_current_ma(std::error_code &ec) {
    return read_s16_le((uint8_t)Register::AverageCurrent, ec);
  }

  /**
   * @brief Get the average power.
   * @details A positive value indicates charging. A negative value indicates
   *          discharging.
   * @param ec Error code set if an error occurs.
   * @return The average power in mW (signed).
   */
  int16_t get_average_power_mw(std::error_code &ec) {
    return read_s16_le((uint8_t)Register::AveragePower, ec);
  }

  /**
   * @brief Get the battery temperature.
   * @details The BQ27220 reports temperature in units of 0.1 Kelvin. This is
   *          converted to degrees Celsius.
   * @param ec Error code set if an error occurs.
   * @return The battery temperature in degrees Celsius.
   */
  float get_temperature_celsius(std::error_code &ec) {
    uint16_t raw = read_u16_le((uint8_t)Register::Temperature, ec);
    if (ec) {
      return 0.0f;
    }
    ec.clear();
    return (float)raw * 0.1f - 273.15f;
  }

  /**
   * @brief Get the battery state of charge.
   * @details This is the percentage of battery charge remaining.
   * @param ec Error code set if an error occurs.
   * @return The battery state of charge in % (0-100).
   */
  uint8_t get_state_of_charge(std::error_code &ec) {
    uint16_t data = read_u16_le((uint8_t)Register::StateOfCharge, ec);
    if (ec) {
      return 0;
    }
    ec.clear();
    return (uint8_t)(data & 0xFF);
  }

  /**
   * @brief Get the battery state of health.
   * @param ec Error code set if an error occurs.
   * @return The battery state of health in % (0-100).
   */
  uint8_t get_state_of_health(std::error_code &ec) {
    uint16_t data = read_u16_le((uint8_t)Register::StateOfHealth, ec);
    if (ec) {
      return 0;
    }
    ec.clear();
    return (uint8_t)(data & 0xFF);
  }

  /**
   * @brief Get the remaining battery capacity.
   * @param ec Error code set if an error occurs.
   * @return The remaining battery capacity in mAh.
   */
  uint16_t get_remaining_capacity_mah(std::error_code &ec) {
    return read_u16_le((uint8_t)Register::RemainingCapacity, ec);
  }

  /**
   * @brief Get the full charge capacity.
   * @param ec Error code set if an error occurs.
   * @return The full charge capacity in mAh.
   */
  uint16_t get_full_charge_capacity_mah(std::error_code &ec) {
    return read_u16_le((uint8_t)Register::FullChargeCapacity, ec);
  }

  /**
   * @brief Get the design capacity.
   * @param ec Error code set if an error occurs.
   * @return The design capacity in mAh.
   */
  uint16_t get_design_capacity_mah(std::error_code &ec) {
    return read_u16_le((uint8_t)Register::DesignCapacity, ec);
  }

  /**
   * @brief Get the estimated time until the battery is empty.
   * @param ec Error code set if an error occurs.
   * @return The time to empty in minutes.
   */
  uint16_t get_time_to_empty_minutes(std::error_code &ec) {
    return read_u16_le((uint8_t)Register::TimeToEmpty, ec);
  }

  /**
   * @brief Get the estimated time until the battery is fully charged.
   * @param ec Error code set if an error occurs.
   * @return The time to full in minutes.
   */
  uint16_t get_time_to_full_minutes(std::error_code &ec) {
    return read_u16_le((uint8_t)Register::TimeToFull, ec);
  }

  /**
   * @brief Get the battery cycle count.
   * @param ec Error code set if an error occurs.
   * @return The number of charge cycles.
   */
  uint16_t get_cycle_count(std::error_code &ec) {
    return read_u16_le((uint8_t)Register::CycleCount, ec);
  }

  /**
   * @brief Get the battery status flags.
   * @param ec Error code set if an error occurs.
   * @return The battery status flags.
   */
  uint16_t get_battery_status(std::error_code &ec) {
    return read_u16_le((uint8_t)Register::BatteryStatus, ec);
  }

protected:
  /// @brief BQ27220 register (standard command) map.
  enum class Register : uint8_t {
    Control = 0x00,             ///< Control() command / status
    AtRate = 0x02,              ///< AtRate() (signed mA)
    AtRateTimeToEmpty = 0x04,   ///< AtRateTimeToEmpty() (minutes)
    Temperature = 0x06,         ///< Temperature() (unsigned, units 0.1 Kelvin)
    Voltage = 0x08,             ///< Voltage() (mV)
    BatteryStatus = 0x0A,       ///< BatteryStatus() (flags)
    Current = 0x0C,             ///< Current() (signed mA)
    RemainingCapacity = 0x10,   ///< RemainingCapacity() (mAh)
    FullChargeCapacity = 0x12,  ///< FullChargeCapacity() (mAh)
    AverageCurrent = 0x14,      ///< AverageCurrent() (signed mA)
    TimeToEmpty = 0x16,         ///< TimeToEmpty() (minutes)
    TimeToFull = 0x18,          ///< TimeToFull() (minutes)
    StandbyCurrent = 0x1A,      ///< StandbyCurrent() (signed mA)
    AveragePower = 0x24,        ///< AveragePower() (signed mW)
    InternalTemperature = 0x28, ///< InternalTemperature() (unsigned, 0.1 Kelvin)
    CycleCount = 0x2A,          ///< CycleCount()
    StateOfCharge = 0x2C,       ///< StateOfCharge() (percent)
    StateOfHealth = 0x2E,       ///< StateOfHealth() (percent)
    DesignCapacity = 0x3C,      ///< DesignCapacity() (mAh)
  };

  /**
   * @brief Read a 16-bit little-endian unsigned value from a register.
   * @details The BQ27220 reports all data values as 16-bit little-endian, so
   *          we cannot use BasePeripheral::read_u16_from_register (which is
   *          big-endian).
   * @param reg The register (command) to read from.
   * @param ec Error code set if an error occurs.
   * @return The 16-bit unsigned value.
   */
  uint16_t read_u16_le(uint8_t reg, std::error_code &ec) {
    uint8_t buf[2] = {0, 0};
    read_many_from_register(reg, buf, 2, ec);
    if (ec) {
      return 0;
    }
    ec.clear();
    return static_cast<uint16_t>(buf[0] | (buf[1] << 8));
  }

  /**
   * @brief Read a 16-bit little-endian signed value from a register.
   * @param reg The register (command) to read from.
   * @param ec Error code set if an error occurs.
   * @return The 16-bit signed value.
   */
  int16_t read_s16_le(uint8_t reg, std::error_code &ec) {
    return static_cast<int16_t>(read_u16_le(reg, ec));
  }
};
} // namespace espp
