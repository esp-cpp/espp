#pragma once

#include <atomic>
#include <cstdint>

#include <sdkconfig.h>

#include "magnetic_encoder_base.hpp"

namespace espp {
/// @brief Enum class for the interface type of the MT6701.
enum class Mt6701Interface : uint8_t {
  I2C = 0, ///< Inter-Integrated Circuit (I2C)
  SSI = 1, ///< Synchronous Serial Interface (SSI), which can be SPI or SSI
};

/// @brief Whether the Mt6701 drives its update loop with a HighResolutionTimer
///        (true) or an espp::Timer (false). Selected by Kconfig / menuconfig.
#if defined(CONFIG_MT6701_USE_HIGH_RESOLUTION_TIMER)
inline constexpr bool mt6701_use_high_resolution_timer = true;
#else
inline constexpr bool mt6701_use_high_resolution_timer = false;
#endif

/**
 * @brief Class for position and velocity measurement using a MT6701 magnetic
 *        encoder. This class starts its own measurement task at the specified
 *        frequency which reads the current angle, updates the accumulator, and
 *        filters / updates the velocity measurement. The Mt6701 supports I2C,
 *        SSI, ABZ, UVW, Analog/PWM, and Push-Button interfaces.
 *
 * This component can be configured to automatically update within its own
 * timer (a HighResolutionTimer by default, changeable to an espp::Timer via
 * KConfig / menuconfig), or if you do not configure it to manage its own timer,
 * then you can call update() within your own function to update the state of
 * the encoder.
 *
 * @warning You should not call update() if you have configured the encoder to
 *          use its own timer or if you have called start() yourself.
 *
 * @note This implementation currently only supports I2C and SSI interfaces.
 *
 * @note The MT6701 has 14-bit angular resolution (16384 counts / revolution).
 *
 * @note There is an implicit assumption in this class regarding the maximum
 *       velocity it can measure (above which there will be aliasing). The
 *       fastest velocity it can measure will be (0.5f / update_period * 60.0f)
 *       RPM which is half a rotation in one update period.
 *
 * @note The assumption above also affects the reliability of the accumulator,
 *       since it is based on accumulating position differences every update
 *       period.
 *
 * \section mt6701_ex1 Mt6701 I2C Example
 * \snippet mt6701_example.cpp mt6701 i2c example
 * \section mt6701_ex2 Mt6701 SSI / SPI Example
 * \snippet mt6701_example.cpp mt6701 ssi example
 */
template <Mt6701Interface Interface = Mt6701Interface::I2C>
class Mt6701 : public MagneticEncoderBase<Mt6701<Interface>, mt6701_use_high_resolution_timer,
                                          16384, CONFIG_MT6701_MIN_DIFF, uint8_t,
                                          Interface == Mt6701Interface::I2C> {
  // Since the base class is a dependent base (its template parameters depend on
  // ours), we bring in the base / grand-base members we use with `using`
  // declarations, otherwise we would have to scope each call with `this->`. This
  // is needed because of the two-phase name lookup for templates.
  using Base =
      MagneticEncoderBase<Mt6701<Interface>, mt6701_use_high_resolution_timer, 16384,
                          CONFIG_MT6701_MIN_DIFF, uint8_t, Interface == Mt6701Interface::I2C>;
  using Base::base_mutex_;
  using Base::count_;
  using Base::logger_;
  using Base::read; // BasePeripheral's buffer read, used by the SSI read() below
  using Base::read_u8_from_register;
  using Base::set_address;
  using Base::set_read;
  using Base::set_write;

  // Allow the base to invoke our (protected) read() via CRTP static dispatch.
  friend Base;

public:
  static constexpr uint8_t DEFAULT_ADDRESS =
      (0b0000110); ///< I2C address of the MT6701. It can be programmed to be 0b1000110 as well.
                   ///< Only used if Interface == Mt6701Interface::I2C.

  using typename Base::velocity_filter_fn;

  /**
   * @brief Enum class for the magnetic field strength of the MT6701.
   */
  enum class MagneticFieldStrength : uint8_t {
    NORMAL = 0,     ///< The magnetic field is normal.
    TOO_STRONG = 1, ///< The magnetic field is too strong to measure the angle.
    TOO_WEAK = 2,   ///< The magnetic field is too weak to measure the angle.
  };

  /**
   * @brief Enum class for the tracking status of the MT6701.
   */
  enum class TrackingStatus : uint8_t {
    NORMAL = 0, ///< Normal tracking status.
    LOST = 1,   ///< Tracking has been lost.
  };

  /**
   * @brief Configuration information for the Mt6701.
   */
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address of the device. Only used if
                                              ///< Interface == Mt6701Interface::I2C.
    BasePeripheral<uint8_t, Interface == Mt6701Interface::I2C>::write_fn write{
        nullptr}; ///< Function to write to the device.
    BasePeripheral<uint8_t, Interface == Mt6701Interface::I2C>::read_fn read{
        nullptr};                                ///< Function to read data from the device.
    velocity_filter_fn velocity_filter{nullptr}; ///< Function to filter the veolcity. @note Will be
                                                 ///< called once every update_period seconds.
    std::chrono::duration<float> update_period{
        .01f}; ///< Update period (1/sample rate) in seconds. This determines the periodicity of the
               ///< timer which will read the position, update the accumulator, and update/filter
               ///< velocity.
    bool auto_init{true}; ///< Whether to automatically initialize the accumulator to the current
                          ///< position on startup.
    bool run_task{true};  ///< Whether to run the timer on startup. If
                          ///< false, you must call update() manually.
    Logger::Verbosity log_level{Logger::Verbosity::WARN};
  };

  /**
   * @brief Construct the Mt6701 and start the update timer if auto_init and run_task are true.
   * @param config Configuration for the Mt6701.
   */
  explicit Mt6701(const Config &config)
      : Base("Mt6701", config.velocity_filter, config.update_period, config.log_level) {
    if constexpr (Interface == Mt6701Interface::I2C) {
      set_address(config.device_address);
      set_write(config.write);
      set_read(config.read);
    } else {
      set_read(config.read);
    }
    if (config.auto_init) {
      std::error_code ec;
      this->initialize(config.run_task, ec);
    }
  }

  /// @brief Stop the update timer before the object is destroyed.
  ~Mt6701() { this->stop(); }

  /**
   * @brief Return the magnetic field strength of the encoder.
   * @return Magnetic field strength of the encoder.
   * @note This function is only available when using SSI communications.
   */
  MagneticFieldStrength get_magnetic_field_strength() const
      requires(Interface == Mt6701Interface::SSI) {
    return magnetic_field_strength_.load();
  }

  /**
   * @brief Return the tracking status of the encoder.
   * @return Tracking status of the encoder.
   * @note This function is only available when using SSI communications.
   */
  TrackingStatus get_tracking_status() const requires(Interface == Mt6701Interface::SSI) {
    return tracking_status_.load();
  }

  /**
   * @brief Return whether the push button is currently pressed.
   * @return True if the push button is pressed, false otherwise.
   * @note This function is only available when using SSI communications.
   */
  bool get_push_button() const requires(Interface == Mt6701Interface::SSI) {
    return push_button_.load();
  }

protected:
  void read(std::error_code &ec) requires(Interface == Mt6701Interface::I2C) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // read the angle count registers
    uint8_t angle_h = read_u8_from_register((uint8_t)Registers::ANGLE_H, ec);
    if (ec) {
      logger_.error_rate_limited("Error reading: {}", ec.message());
      return;
    }
    uint8_t angle_l = read_u8_from_register((uint8_t)Registers::ANGLE_L, ec) >> 2;
    if (ec) {
      logger_.error_rate_limited("Error reading: {}", ec.message());
      return;
    }
    count_ = ((angle_h << 6) | angle_l);
  }

  void read(std::error_code &ec) requires(Interface == Mt6701Interface::SSI) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // read the angle count as 24 bits (3 bytes) from the serial stream
    uint8_t buffer[3] = {0};
    read(&buffer[0], 3, ec);
    if (ec) {
      logger_.error_rate_limited("Error reading: {}", ec.message());
      return;
    }
    // the first 14 bits are the angle data, followed by 4 bit status, and 6 bit
    // crc
    uint16_t angle_h = buffer[0];
    uint8_t angle_l = buffer[1] >> 2;
    uint16_t raw_count = (angle_h << 6) | angle_l;
    // status is the lower 2 bits of the second byte and the upper 2 bits of
    // the third byte
    uint8_t status = ((buffer[1] & 0b11) << 2) | (buffer[2] >> 6);
    // crc is the lower 6 bits of the third byte
    uint8_t crc = buffer[2] & 0b111111;
    // The CRC is computed over the 18 data bits (14-bit angle + 4-bit status).
    // NOTE: this is currently observability-only (a mismatch is logged but the
    // sample is still used) since the CRC implementation has not been verified
    // against hardware; it can be promoted to rejecting the sample once verified.
    uint8_t expected_crc = crc6((static_cast<uint32_t>(raw_count) << 4) | (status & 0x0F));
    if (crc != expected_crc) {
      logger_.warn_rate_limited("CRC mismatch: got {:#04x}, expected {:#04x}", crc, expected_crc);
    }
    logger_.debug("Angle: {}, Status: {}, CRC: {}", raw_count, status, crc);
    // update the count
    count_ = raw_count;
    // update the magnetic field strength, tracking status, and push button.
    // strength is the lower two bits [0:1], push button is the third bit [2],
    // and tracking status is the fourth bit [3]
    magnetic_field_strength_ = (MagneticFieldStrength)(status & 0b11);
    push_button_ = (bool)((status >> 2) & 0b1);
    tracking_status_ = (TrackingStatus)((status >> 3) & 0b1);
  }

  /// @brief MT6701 SSI CRC-6 (polynomial x^6 + x + 1) over the 18-bit payload.
  /// @param data18 The 18-bit payload: (14-bit angle << 4) | 4-bit status.
  /// @return The computed 6-bit CRC.
  static uint8_t crc6(uint32_t data18) {
    static constexpr uint8_t table[64] = {
        0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09, 0x18, 0x1B, 0x1E, 0x1D, 0x14,
        0x17, 0x12, 0x11, 0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39, 0x28, 0x2B,
        0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21, 0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29,
        0x2A, 0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32, 0x13, 0x10, 0x15, 0x16,
        0x1F, 0x1C, 0x19, 0x1A, 0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02};
    uint8_t crc = table[(data18 >> 12) & 0x3F];
    crc = table[crc ^ ((data18 >> 6) & 0x3F)];
    crc = table[crc ^ (data18 & 0x3F)];
    return crc;
  }

  /**
   * @brief Register map for the MT6701.
   *
   * @note The MT6701 contains a push-button output (pin 5) with configuration
   *       (mentioned on page 25) via PUSH_THRD register, PUSH_DIFF_DLY
   *       register, and PUSH_TIME_OUT register. However, the register addresses
   *       for these configurations (and their bitfields) are not provided in
   *       the datasheet and must be provided by the manufacturer.
   *
   * @note The push button can only be read from the MT6701 when using SSI
   *       communications, and is returned as part of the magnetic field status
   *       truth table (page 26).
   */
  enum class Registers : uint8_t {
    ANGLE_H = 0x03,     ///< Angle[13:6]
    ANGLE_L = 0x04,     ///< Angle[5:0] (bit 2-7)
    MUX_1 = 0x25,       ///< UVW MUX (bit 7)
    MUX_2 = 0x29,       ///< ABZ MUX (bit 6), DIR (bit 1)
    RES_1 = 0x30,       ///< UVW_RES (bit 4-7), ABZ_RES[9:8] (bit 0-1)
    RES_2 = 0x31,       ///< ABZ_RES[7:0]
    CONFIG_1 = 0x32,    ///< HYST[2] (bit 7), Z_PULSE_WIDTH (bit 4-6), ZERO[11:8] (bit 0-3)
    CONFIG_2 = 0x33,    ///< ZERO[7:0]
    CONFIG_3 = 0x34,    ///< HYST[1:0] (bit 6-7)
    CONFIG_4 = 0x38,    ///< PWM_FREQ (bit 7), PWM_POL (bit 6), OUT_MODE (bit 5)
    A_SS_HIGH = 0x3E,   ///< A_STOP[11:8] (bit 4-7), A_START[11:8] (bit 0-3)
    A_START_LOW = 0x3F, ///< A_START[7:0]
    A_STOP_LOW = 0x40,  ///< A_STOP[7:0]
  };

  std::atomic<MagneticFieldStrength> magnetic_field_strength_{MagneticFieldStrength::NORMAL};
  std::atomic<TrackingStatus> tracking_status_{TrackingStatus::NORMAL};
  std::atomic<bool> push_button_{false};
};
} // namespace espp

// for easy printing of MagneticFieldStatus using libfmt
template <> struct fmt::formatter<espp::Mt6701<espp::Mt6701Interface::SSI>::MagneticFieldStrength> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::Mt6701<espp::Mt6701Interface::SSI>::MagneticFieldStrength const &mfs,
              FormatContext &ctx) const {
    return fmt::format_to(
        ctx.out(), "{}",
        mfs == espp::Mt6701<espp::Mt6701Interface::SSI>::MagneticFieldStrength::NORMAL ? "NORMAL"
        : mfs == espp::Mt6701<espp::Mt6701Interface::SSI>::MagneticFieldStrength::TOO_STRONG
            ? "TOO_STRONG"
            : "TOO_WEAK");
  }
};
template <> struct fmt::formatter<espp::Mt6701<espp::Mt6701Interface::SSI>::TrackingStatus> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::Mt6701<espp::Mt6701Interface::SSI>::TrackingStatus const &ts,
              FormatContext &ctx) const {
    return fmt::format_to(
        ctx.out(), "{}",
        ts == espp::Mt6701<espp::Mt6701Interface::SSI>::TrackingStatus::NORMAL ? "NORMAL" : "LOST");
  }
};
template <> struct fmt::formatter<espp::Mt6701<espp::Mt6701Interface::I2C>::MagneticFieldStrength> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::Mt6701<espp::Mt6701Interface::I2C>::MagneticFieldStrength const &mfs,
              FormatContext &ctx) const {
    return fmt::format_to(
        ctx.out(), "{}",
        mfs == espp::Mt6701<espp::Mt6701Interface::I2C>::MagneticFieldStrength::NORMAL ? "NORMAL"
        : mfs == espp::Mt6701<espp::Mt6701Interface::I2C>::MagneticFieldStrength::TOO_STRONG
            ? "TOO_STRONG"
            : "TOO_WEAK");
  }
};
template <> struct fmt::formatter<espp::Mt6701<espp::Mt6701Interface::I2C>::TrackingStatus> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::Mt6701<espp::Mt6701Interface::I2C>::TrackingStatus const &ts,
              FormatContext &ctx) const {
    return fmt::format_to(
        ctx.out(), "{}",
        ts == espp::Mt6701<espp::Mt6701Interface::I2C>::TrackingStatus::NORMAL ? "NORMAL" : "LOST");
  }
};
