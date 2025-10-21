#pragma once

#include <cstdint>

#include "base_peripheral.hpp"

namespace espp {
/**
 * INA226 Current/Power Monitor (I2C)
 *
 * Provides helpers to configure and read bus voltage, shunt voltage, current,
 * and power. Uses BasePeripheral for I2C access, 8-bit register addresses.
 *
 * Section: Example
 * \snippet ina226_example.cpp ina226 example
 */
class Ina226 : public BasePeripheral<uint8_t, true> {
public:
  /// Address is configured based on whether A0/A1 are connected to GND, VCC,
  /// SDA, or SCL, address can be 0x40..0x4F
  static constexpr uint8_t DEFAULT_ADDRESS = 0x40; ///< Default I2C address if A0/A1=GND

  static constexpr uint16_t MANUFACTURER_ID_TI = 0x5449; ///< Texas Instruments Manufacturer ID
  static constexpr uint16_t DIE_ID_INA226 = 0x2260;      ///< INA226 Die ID

  /// Averaging (AVG) field values (bits 14..12 of CONFIG)
  enum class Avg : uint16_t {
    AVG_1 = 0, // NOTE: default on chip reset
    AVG_4 = 1,
    AVG_16 = 2,
    AVG_64 = 3,
    AVG_128 = 4,
    AVG_256 = 5,
    AVG_512 = 6,
    AVG_1024 = 7,
  };

  /// Bus/Shunt conversion time (VBUSCT/VSHCT) field values (bits 11..9, 8..6)
  enum class ConvTime : uint16_t {
    US_140 = 0,
    US_204 = 1,
    US_332 = 2,
    US_588 = 3,
    MS_1_1 = 4, // 1.1 ms, NOTE: this is the default on chip reset
    MS_2_116 = 5,
    MS_4_156 = 6,
    MS_8_244 = 7,
  };

  /// Operating mode (MODE bits 2..0)
  enum class Mode : uint16_t {
    POWER_DOWN = 0,
    SHUNT_TRIG = 1,
    BUS_TRIG = 2,
    SHUNT_BUS_TRIG = 3,
    ADC_OFF = 4,
    SHUNT_CONT = 5,
    BUS_CONT = 6,
    SHUNT_BUS_CONT = 7, // NOTE: default on chip reset
  };

  /// Configuration structure for INA226
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address, 0x40..0x4F
    // Measurement/config
    Avg averaging = Avg::AVG_16;                 // Averaging mode
    ConvTime bus_conv_time = ConvTime::MS_1_1;   ///< Bus conversion time
    ConvTime shunt_conv_time = ConvTime::MS_1_1; ///< Shunt conversion time
    Mode mode = Mode::SHUNT_BUS_CONT;            ///< Operating mode
    // Current calibration parameters
    // current_lsb in Amps/LSB, shunt_resistance in Ohms
    float current_lsb = 0.001f;         // 1 mA/LSB default
    float shunt_resistance_ohms = 0.1f; // 0.1 ohm default
    // I/O hooks
    BasePeripheral::probe_fn probe{nullptr}; ///< Probe function to check I2C device presence
    BasePeripheral::write_fn write{nullptr}; ///< Write function to send data to I2C device
    BasePeripheral::read_register_fn read_register{
        nullptr}; ///< Read function to read a register from I2C device
    BasePeripheral::write_then_read_fn write_then_read{
        nullptr}; ///< Write then read function for I2C device
    // Behavior
    bool auto_init = true; ///< Automatically initialize on construction
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log level for INA226 messages
  };

  /// Constructor with configuration
  /// @param config Configuration structure with device address, averaging,
  ///               conversion times, mode, current LSB, shunt resistance, and
  ///               I/O hooks
  explicit Ina226(const Config &config)
      : BasePeripheral({.address = config.device_address,
                        .probe = config.probe,
                        .write = config.write,
                        .read_register = config.read_register,
                        .write_then_read = config.write_then_read},
                       "Ina226", config.log_level)
      , config_(config) {
    if (config.auto_init) {
      std::error_code ec;
      if (!initialize(ec)) {
        logger_.error("Failed to initialize INA226: {}", ec.message());
      }
    }
  }

  /// Initialize the INA226
  /// @param ec Error code to capture any initialization errors
  /// @return true if initialization succeeded, false if it failed
  bool initialize(std::error_code &ec) { return init(config_, ec); }

  // Identification helpers

  /// Read manufacturer ID (0x5449 for Texas Instruments)
  /// @param ec Error code to capture any read errors
  /// @return Manufacturer ID (0x5449) or 0 on error
  uint16_t manufacturer_id(std::error_code &ec) const {
    return read_u16_from_register((uint8_t)Reg::MANUFACTURER_ID, ec);
  }
  /// Read die ID (0x2260 for INA226)
  /// @param ec Error code to capture any read errors
  /// @return Die ID (0x2260) or 0 on error
  uint16_t die_id(std::error_code &ec) const {
    return read_u16_from_register((uint8_t)Reg::DIE_ID, ec);
  }

  // Engineering-unit helpers

  /// Read shunt voltage in volts
  /// @param ec Error code to capture any read errors
  /// @return Shunt voltage in volts, or 0.0f on error
  /// @note The shunt voltage is signed, so it can be negative if the current
  ///       flows in the reverse direction. The LSB is 2.5 uV
  float shunt_voltage_volts(std::error_code &ec) const {
    int16_t raw = read_shunt_raw(ec);
    if (ec)
      return 0.0f;
    return raw * 2.5e-6f;
  }

  /// Read bus voltage in volts
  /// @param ec Error code to capture any read errors
  /// @return Bus voltage in volts, or 0.0f on error
  /// @note The bus voltage is unsigned, so it cannot be negative. The LSB is
  ///       1.25 mV
  float bus_voltage_volts(std::error_code &ec) const {
    uint16_t raw = read_bus_raw(ec);
    if (ec)
      return 0.0f;
    // each LSB is 1.25mV
    return (raw * 1.25e-3f);
  }

  /// Read current in amps
  /// @param ec Error code to capture any read errors
  /// @return Current in amps, or 0.0f on error
  /// @note The current is signed, so it can be negative if the current flows in
  ///       the reverse direction. The LSB is set via the calibrate() method and
  ///       is in Amps/LSB.
  float current_amps(std::error_code &ec) const {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    int16_t raw = read_current_raw(ec);
    if (ec)
      return 0.0f;
    return raw * current_lsb_;
  }

  /// Read power in watts
  /// @param ec Error code to capture any read errors
  /// @return Power in watts, or 0.0f on error
  /// @note The power is unsigned, so it cannot be negative. The LSB is 25 *
  ///       current_lsb, where current_lsb is set via the calibrate() method.
  float power_watts(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint16_t raw = read_power_raw(ec);
    if (ec)
      return 0.0f;
    return raw * (25.0f * current_lsb_);
  }

  /// Reset the INA226 to default settings
  /// @param ec Error code to capture any write errors
  /// @return true if reset succeeded, false if it failed
  bool reset(std::error_code &ec) {
    // Set bit 15 of CONFIG to reset
    static constexpr uint16_t RESET_BIT = 1 << 15;
    uint16_t word = RESET_BIT;
    logger_.info("Resetting INA226 to default settings");
    write_u16_to_register((uint8_t)Reg::CONFIG, word, ec);
    return !ec;
  }

  /// Configure the INA226 with averaging, conversion times, and mode
  /// @param avg Averaging mode
  /// @param vbus Bus voltage conversion time
  /// @param vshunt Shunt voltage conversion time
  /// @param mode Operating mode
  /// @param ec Error code to capture any write errors
  /// @return true if configuration succeeded, false if it failed
  bool configure(Avg avg, ConvTime vbus, ConvTime vshunt, Mode mode, std::error_code &ec) {
    // Build config: AVG[14:12], VBUSCT[11:9], VSHCT[8:6], MODE[2:0]
    uint16_t word = 0;
    // NOTE: bit 15 is used to reset, so we don't set it here
    // NOTE: bits 12-14 are not used, and should always be 0b100 << 12
    word |= (static_cast<uint16_t>(avg) & 0x7) << 9;
    word |= (static_cast<uint16_t>(vbus) & 0x7) << 6;
    word |= (static_cast<uint16_t>(vshunt) & 0x7) << 3;
    word |= (static_cast<uint16_t>(mode) & 0x7);
    write_u16_to_register((uint8_t)Reg::CONFIG, word, ec);
    return !ec;
  }

  /// Calibrate the INA226 with current LSB and shunt resistance
  /// @param current_lsb Current LSB in Amps/LSB
  /// @param shunt_res_ohms Shunt resistance in Ohms
  /// @param ec Error code to capture any write errors
  /// @return true if calibration succeeded, false if it failed
  /// @note This function programs the CALIBRATION register based on the
  ///       provided current_lsb and shunt resistance. The current_lsb is used
  ///       to determine the scaling of the CURRENT and POWER registers. The
  ///       shunt resistance is used to calculate the calibration value.
  /// @note The current_lsb should be chosen based on the expected maximum
  ///       current and the resolution required. A smaller current_lsb provides
  ///       higher resolution but reduces the maximum measurable current.
  /// @note Per the datasheet, the calibration value is calculated as:
  ///       \f[
  ///       Cal = floor(0.00512 / (current_lsb * shunt_res_ohms))
  ///       \f]
  ///       This ensures that the calibration value is non-zero and avoids
  ///       division by zero. If the calculated value is zero, it is set to 1 to
  ///       avoid issues. This function should be called after configuring the
  ///       INA226.
  bool calibrate(float current_lsb, float shunt_res_ohms, std::error_code &ec) {
    // Set calibration based on current_lsb (A/LSB) and shunt resistance (Ohms)
    // CAL = floor(0x8000 / (current_lsb * Rshunt)) per datasheet (5120/ (curr_lsb*R) is common for
    // INA219, but INA226 uses 0.00512/ (curr_lsb*R) scaled for 16-bit: 0.00512 / (A/LSB * Ohms))
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    current_lsb_ = current_lsb;
    shunt_res_ohms_ = shunt_res_ohms;
    // Datasheet: Cal = 0.00512 / (current_lsb * Rshunt)
    float cal_f = 0.00512f / (current_lsb_ * shunt_res_ohms_);
    uint16_t cal = (uint16_t)(cal_f + 0.5f);
    if (cal == 0)
      cal = 1; // avoid zero
    write_u16_to_register((uint8_t)Reg::CALIBRATION, cal, ec);
    return !ec;
  }

protected:
  // Register map (datasheet)
  enum class Reg : uint8_t {
    CONFIG = 0x00,
    SHUNT_VOLTAGE = 0x01, // 16-bit signed, 2.5uV/LSB
    BUS_VOLTAGE = 0x02,   // 16-bit unsigned, 1.25mV/LSB
    POWER = 0x03,         // 16-bit unsigned, 25*CURRENT_LSB per LSB
    CURRENT = 0x04,       // 16-bit signed, CURRENT_LSB per LSB
    CALIBRATION = 0x05,   // 16-bit unsigned
    MASK_ENABLE = 0x06,
    ALERT_LIMIT = 0x07,
    MANUFACTURER_ID = 0xFE, // 0x5449
    DIE_ID = 0xFF,          // 0x2260
  };

  bool init(const Config &c, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // read manufacturer and die ID to verify presence
    uint16_t manufacturer = manufacturer_id(ec);
    if (ec)
      return false;
    if (manufacturer != MANUFACTURER_ID_TI) {
      logger_.error("INA226 manufacturer ID mismatch: expected 0x{:04X}, got 0x{:04X}",
                    MANUFACTURER_ID_TI, manufacturer);
      ec = make_error_code(std::errc::no_such_device);
      return false;
    }
    uint16_t die = die_id(ec);
    if (ec)
      return false;
    if (die != DIE_ID_INA226) {
      logger_.error("INA226 die ID mismatch: expected 0x{:04X}, got 0x{:04X}", DIE_ID_INA226, die);
      ec = make_error_code(std::errc::no_such_device);
      return false;
    }
    // if here, device is present, so reset it
    if (!reset(ec))
      return false;
    // Program config
    configure(c.averaging, c.bus_conv_time, c.shunt_conv_time, c.mode, ec);
    if (ec)
      return false;
    // Program calibration
    calibrate(c.current_lsb, c.shunt_resistance_ohms, ec);
    if (ec)
      return false;
    // Cache
    current_lsb_ = c.current_lsb;
    shunt_res_ohms_ = c.shunt_resistance_ohms;
    return true;
  }

  // Raw register reads (signed/unsigned as appropriate by datasheet)
  int16_t read_shunt_raw(std::error_code &ec) const {
    return (int16_t)read_u16_from_register((uint8_t)Reg::SHUNT_VOLTAGE, ec);
  }
  uint16_t read_bus_raw(std::error_code &ec) const {
    return read_u16_from_register((uint8_t)Reg::BUS_VOLTAGE, ec);
  }
  int16_t read_current_raw(std::error_code &ec) const {
    return (int16_t)read_u16_from_register((uint8_t)Reg::CURRENT, ec);
  }
  uint16_t read_power_raw(std::error_code &ec) const {
    return read_u16_from_register((uint8_t)Reg::POWER, ec);
  }

  Config config_{};
  float current_lsb_{0.001f};
  float shunt_res_ohms_{0.1f};
};
} // namespace espp
