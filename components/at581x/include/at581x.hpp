#pragma once

#include <array>
#include <functional>
#include <span>
#include <system_error>

#include "base_peripheral.hpp"

namespace espp {
/// @brief Driver for the AT581X (AirTouch) 5.8 GHz microwave radar
///        human-presence / motion sensor.
///
/// The AT581X is configured over I2C, while presence/motion is reported on a
/// dedicated active-high output GPIO (which stays asserted for the configured
/// "trigger keep" time after the last detection). This driver handles the I2C
/// configuration of the chip (detection distance/sensitivity, RF frequency,
/// gain, power, and timing); to react to presence, attach an interrupt (e.g.
/// `espp::Interrupt`) to the radar's output GPIO. See the example.
///
/// The AT581X is the radar found on modules such as the MoreSense
/// `MS58-3909S68U4` used on the ESP32-S3-BOX-3 sensor / dock board (I2C on
/// SDA=GPIO41, SCL=GPIO40 there).
///
/// The register protocol implemented here follows the AT581X datasheet and the
/// reference drivers from Espressif and ESPHome.
///
/// \section at581x_ex1 AT581X Example
/// \snippet at581x_example.cpp at581x example
class At581x : public BasePeripheral<> {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x28; ///< Default I2C address of the AT581X.

  /// @brief Configuration for the AT581X driver.
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS;       ///< I2C address of the device.
    BasePeripheral::write_fn write;                 ///< Function for writing to the device.
    BasePeripheral::read_register_fn read_register; ///< Function for reading a register.
    int frequency_mhz = 5800;   ///< RF frequency in MHz. Must be one of allowed_frequencies_mhz().
    int sensing_distance = 823; ///< Detection distance, 0..1023. Larger = farther/more sensitive.
    int gain = 3;               ///< Gain stage index, 0..12. Higher = more gain.
    int power_consumption_ua = 70;   ///< Power draw in µA. Must be one of allowed_power_ua().
    int trigger_base_time_ms = 500;  ///< Base detection window in ms.
    int trigger_keep_time_ms = 1500; ///< How long the output stays asserted after detection, ms.
    int protect_time_ms = 1000;      ///< Protection (re-trigger lockout) time in ms.
    int poweron_selfcheck_time_ms = 2000; ///< Power-on self-check time in ms (0..65535).
    bool auto_init = true; ///< If true, write the configuration to the chip on construction.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity.
  };

  /// @brief Construct an AT581X driver.
  /// @param config The configuration for the driver.
  explicit At581x(const Config &config)
      : BasePeripheral({.address = config.device_address,
                        .write = config.write,
                        .read_register = config.read_register},
                       "At581x", config.log_level)
      , frequency_mhz_(config.frequency_mhz)
      , sensing_distance_(config.sensing_distance)
      , gain_(config.gain)
      , power_consumption_ua_(config.power_consumption_ua)
      , trigger_base_time_ms_(config.trigger_base_time_ms)
      , trigger_keep_time_ms_(config.trigger_keep_time_ms)
      , protect_time_ms_(config.protect_time_ms)
      , poweron_selfcheck_time_ms_(config.poweron_selfcheck_time_ms) {
    if (config.auto_init) {
      std::error_code ec;
      initialize(ec);
      if (ec) {
        logger_.error("Failed to initialize: {}", ec.message());
      }
    }
  }

  /// @brief Initialize the AT581X by writing the current configuration and resetting it.
  /// @param ec Set on error.
  /// @return True on success.
  bool initialize(std::error_code &ec) { return write_config(ec); }

  /// @brief Write the full configuration to the chip and reset its RF frontend so it takes effect.
  /// @param ec Set on error.
  /// @return True on success.
  bool write_config(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("Writing config: freq={}MHz, sensing_distance={}, gain={}, power={}uA, "
                 "trigger_base={}ms, trigger_keep={}ms, protect={}ms, selfcheck={}ms",
                 frequency_mhz_, sensing_distance_, gain_, power_consumption_ua_,
                 trigger_base_time_ms_, trigger_keep_time_ms_, protect_time_ms_,
                 poweron_selfcheck_time_ms_);

    // 1) Select frequency mode (also marks freq + gain values as present).
    write_u8_to_register(REG_FREQ_MODE, FREQ_MODE_VALUE, ec);
    if (ec)
      return false;

    // 2) Look up the frequency point and write its register values.
    int freq_index = index_of(FREQ_TABLE_MHZ, frequency_mhz_);
    if (freq_index < 0) {
      logger_.error("Invalid frequency {}MHz", frequency_mhz_);
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    write_u8_to_register(REG_FREQ_5F, FREQ_5F_TABLE[freq_index], ec);
    if (ec)
      return false;
    write_u8_to_register(REG_FREQ_60, FREQ_60_TABLE[freq_index], ec);
    if (ec)
      return false;

    // 3) Detection distance / threshold (16-bit, little-endian). Larger distance -> smaller delta.
    int delta = 1023 - sensing_distance_;
    write_u8_to_register(REG_THRESHOLD_LO, static_cast<uint8_t>(delta & 0xFF), ec);
    if (ec)
      return false;
    write_u8_to_register(REG_THRESHOLD_HI, static_cast<uint8_t>((delta >> 8) & 0xFF), ec);
    if (ec)
      return false;

    // 4) Power consumption (bitfields across two registers).
    int power_index = index_of(POWER_TABLE_UA, power_consumption_ua_);
    if (power_index < 0) {
      logger_.error("Invalid power {}uA", power_consumption_ua_);
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    uint8_t pwr_lo = PWR_THRESH_VAL_EN | PWR_WORK_TIME_EN | POWER_67_TABLE[power_index];
    uint8_t pwr_hi = PWR_BURST_TIME_EN | PWR_THRESH_EN | POWER_68_TABLE[power_index];
    write_u8_to_register(REG_POWER_LO, pwr_lo, ec);
    if (ec)
      return false;
    write_u8_to_register(REG_POWER_HI, pwr_hi, ec);
    if (ec)
      return false;

    // 5) Gain stage.
    if (gain_ < 0 || static_cast<size_t>(gain_) >= GAIN_5C_TABLE.size() ||
        static_cast<size_t>(gain_ >> 1) >= GAIN_63_TABLE.size()) {
      logger_.error("Invalid gain index {}", gain_);
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    write_u8_to_register(REG_GAIN_5C, GAIN_5C_TABLE[gain_], ec);
    if (ec)
      return false;
    write_u8_to_register(REG_GAIN_63, GAIN_63_TABLE[gain_ >> 1], ec);
    if (ec)
      return false;

    // 6) Timing parameters.
    write_u32_le(REG_TRIGGER_BASE_TIME, static_cast<uint32_t>(trigger_base_time_ms_), ec);
    if (ec)
      return false;
    write_u32_le(REG_TRIGGER_KEEP_TIME, static_cast<uint32_t>(trigger_keep_time_ms_), ec);
    if (ec)
      return false;
    write_u16_le(REG_PROTECT_TIME, static_cast<uint16_t>(protect_time_ms_), ec);
    if (ec)
      return false;
    write_u16_le(REG_SELF_CHECK_TIME, static_cast<uint16_t>(poweron_selfcheck_time_ms_), ec);
    if (ec)
      return false;

    // 7) Enable the timing output and the chip.
    write_u8_to_register(REG_TIME_ENABLE, 0x01, ec);
    if (ec)
      return false;
    write_u8_to_register(REG_CHIP_ENABLE, 0x04, ec);
    if (ec)
      return false;

    // 8) Reset the RF frontend so the new configuration takes effect.
    return reset(ec);
  }

  /// @brief Reset the AT581X RF frontend (required for new config to take effect).
  /// @param ec Set on error.
  /// @return True on success.
  bool reset(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("Resetting RF frontend");
    write_u8_to_register(REG_RESET, 0, ec);
    if (ec)
      return false;
    write_u8_to_register(REG_RESET, 1, ec);
    return !ec;
  }

  /// @brief Turn the RF / analog frontend on or off (for power saving).
  /// @param enable True to enable RF, false to disable.
  /// @param ec Set on error.
  /// @return True on success.
  bool set_rf_enabled(bool enable, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("{} RF", enable ? "Enabling" : "Disabling");
    const auto &values = enable ? RF_ON_TABLE : RF_OFF_TABLE;
    for (size_t i = 0; i < RF_REG_ADDR.size(); i++) {
      write_u8_to_register(RF_REG_ADDR[i], values[i], ec);
      if (ec)
        return false;
    }
    return true;
  }

  /// @brief Set the detection distance / sensitivity (0..1023, larger = farther) and re-apply.
  /// @param distance The new sensing distance.
  /// @param ec Set on error.
  /// @return True on success.
  bool set_sensing_distance(int distance, std::error_code &ec) {
    sensing_distance_ = distance;
    return write_config(ec);
  }

  /// @brief Set the gain stage index (0..12, higher = more gain) and re-apply.
  /// @param gain The new gain index.
  /// @param ec Set on error.
  /// @return True on success.
  bool set_gain(int gain, std::error_code &ec) {
    gain_ = gain;
    return write_config(ec);
  }

  /// @brief Set how long the output stays asserted after detection (ms) and re-apply.
  /// @param trigger_keep_time_ms The new trigger-keep time in ms.
  /// @param ec Set on error.
  /// @return True on success.
  bool set_trigger_keep_time(int trigger_keep_time_ms, std::error_code &ec) {
    trigger_keep_time_ms_ = trigger_keep_time_ms;
    return write_config(ec);
  }

  /// @brief Get the current configured sensing distance (0..1023).
  int get_sensing_distance() const { return sensing_distance_; }

  /// @brief Get the current configured gain index (0..12).
  int get_gain() const { return gain_; }

  /// @brief The RF frequencies (MHz) accepted by frequency_mhz / set_frequency.
  static std::span<const uint16_t> allowed_frequencies_mhz() {
    return {FREQ_TABLE_MHZ.data(), FREQ_TABLE_MHZ.size()};
  }

  /// @brief The power-consumption values (µA) accepted by power_consumption_ua.
  static std::span<const uint8_t> allowed_power_ua() {
    return {POWER_TABLE_UA.data(), POWER_TABLE_UA.size()};
  }

protected:
  // Write a 16-bit value little-endian across two consecutive registers.
  void write_u16_le(uint8_t reg, uint16_t value, std::error_code &ec) {
    write_u8_to_register(reg, static_cast<uint8_t>(value & 0xFF), ec);
    if (ec)
      return;
    write_u8_to_register(reg + 1, static_cast<uint8_t>((value >> 8) & 0xFF), ec);
  }

  // Write a 32-bit value little-endian across four consecutive registers.
  void write_u32_le(uint8_t reg, uint32_t value, std::error_code &ec) {
    for (int i = 0; i < 4; i++) {
      write_u8_to_register(reg + i, static_cast<uint8_t>((value >> (8 * i)) & 0xFF), ec);
      if (ec)
        return;
    }
  }

  template <typename T, size_t N> static int index_of(const std::array<T, N> &table, int value) {
    for (size_t i = 0; i < N; i++) {
      if (table[i] == value)
        return static_cast<int>(i);
    }
    return -1;
  }

  // --- Register addresses (AT581X datasheet) ---
  static constexpr uint8_t REG_RESET = 0x00;             // write 0 then 1 to reset RF frontend
  static constexpr uint8_t REG_THRESHOLD_LO = 0x10;      // detection threshold, low byte
  static constexpr uint8_t REG_THRESHOLD_HI = 0x11;      // detection threshold, high byte
  static constexpr uint8_t REG_SELF_CHECK_TIME = 0x38;   // 2 bytes
  static constexpr uint8_t REG_TRIGGER_BASE_TIME = 0x3D; // 4 bytes
  static constexpr uint8_t REG_TIME_ENABLE = 0x41;       // enable timing output (0x01)
  static constexpr uint8_t REG_TRIGGER_KEEP_TIME = 0x42; // 4 bytes
  static constexpr uint8_t REG_PROTECT_TIME = 0x4E;      // 2 bytes
  static constexpr uint8_t REG_CHIP_ENABLE = 0x55;       // enable chip (0x04)
  static constexpr uint8_t REG_GAIN_5C = 0x5C;
  static constexpr uint8_t REG_FREQ_5F = 0x5F;
  static constexpr uint8_t REG_FREQ_60 = 0x60;
  static constexpr uint8_t REG_FREQ_MODE = 0x61;
  static constexpr uint8_t REG_GAIN_63 = 0x63;
  static constexpr uint8_t REG_POWER_LO = 0x67;
  static constexpr uint8_t REG_POWER_HI = 0x68;

  // Marks frequency (0x02) and gain (0x08) values as present (| 0xC0).
  static constexpr uint8_t FREQ_MODE_VALUE = 0xCA;

  // Power register bitfield enables.
  static constexpr uint8_t PWR_WORK_TIME_EN = 0x08;  // reg 0x67
  static constexpr uint8_t PWR_BURST_TIME_EN = 0x20; // reg 0x68
  static constexpr uint8_t PWR_THRESH_EN = 0x40;     // reg 0x68
  static constexpr uint8_t PWR_THRESH_VAL_EN = 0x80; // reg 0x67

  // --- Lookup tables (from AT581X datasheet / reference drivers) ---
  static constexpr std::array<uint16_t, 12> FREQ_TABLE_MHZ = {5696, 5715, 5730, 5748, 5765, 5784,
                                                              5800, 5819, 5836, 5851, 5869, 5888};
  static constexpr std::array<uint8_t, 12> FREQ_5F_TABLE = {0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
                                                            0x46, 0x47, 0x40, 0x41, 0x42, 0x43};
  static constexpr std::array<uint8_t, 12> FREQ_60_TABLE = {0x9d, 0x9d, 0x9d, 0x9d, 0x9d, 0x9d,
                                                            0x9d, 0x9d, 0x9e, 0x9e, 0x9e, 0x9e};

  static constexpr std::array<uint8_t, 13> GAIN_5C_TABLE = {
      0x08, 0x18, 0x28, 0x38, 0x48, 0x58, 0x68, 0x78, 0x88, 0x98, 0xa8, 0xb8, 0xc8};
  static constexpr std::array<uint8_t, 7> GAIN_63_TABLE = {0x00, 0x01, 0x02, 0x03,
                                                           0x04, 0x05, 0x06};

  static constexpr std::array<uint8_t, 16> POWER_TABLE_UA = {48, 56, 63, 70, 77, 91, 105, 115,
                                                             40, 44, 47, 51, 54, 61, 68,  78};
  static constexpr std::array<uint8_t, 16> POWER_67_TABLE = {
      0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7};
  static constexpr std::array<uint8_t, 16> POWER_68_TABLE = {0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8,
                                                             24,  24,  24,  24,  24,  24,  24,  24};

  // RF / analog frontend on/off (registers 0x5d, 0x62, 0x51).
  static constexpr std::array<uint8_t, 3> RF_REG_ADDR = {0x5d, 0x62, 0x51};
  static constexpr std::array<uint8_t, 3> RF_ON_TABLE = {0x45, 0x55, 0xA0};
  static constexpr std::array<uint8_t, 3> RF_OFF_TABLE = {0x46, 0xaa, 0x50};

  int frequency_mhz_;
  int sensing_distance_;
  int gain_;
  int power_consumption_ua_;
  int trigger_base_time_ms_;
  int trigger_keep_time_ms_;
  int protect_time_ms_;
  int poweron_selfcheck_time_ms_;
};
} // namespace espp
