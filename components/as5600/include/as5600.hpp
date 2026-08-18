#pragma once

#include <atomic>
#include <cstdint>

#include <sdkconfig.h>

#include "magnetic_encoder_base.hpp"

namespace espp {
/// @brief Whether the As5600 drives its update loop with a HighResolutionTimer
///        (true) or an espp::Timer (false). Selected by Kconfig / menuconfig.
#if defined(CONFIG_AS5600_USE_HIGH_RESOLUTION_TIMER)
inline constexpr bool as5600_use_high_resolution_timer = true;
#else
inline constexpr bool as5600_use_high_resolution_timer = false;
#endif

/**
 * @brief Class for position and velocity measurement using a AS5600 magnetic
 *        encoder. This class starts its own measurement task at the specified
 *        frequency which reads the current angle, updates the accumulator,
 *        and filters / updates the velocity measurement. The datasheet for
 *        the AS5600 can be found here:
 *        https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf/649ee61c-8f9a-20df-9e10-43173a3eb323
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
 * @note The AS5600 has 12-bit angular resolution (4096 counts / revolution).
 *
 * @note There is an implicit assumption in this class regarding the maximum
 *       velocity it can measure (above which there will be aliasing). The
 *       fastest velocity it can measure will be (0.5f / update_period * 60.0f)
 *       which is half a rotation in one update period.
 *
 * @note The assumption above also affects the reliability of the accumulator,
 *       since it is based on accumulating position differences every update
 *       period.
 *
 * \section as5600_ex1 As5600 Example
 * \snippet as5600_example.cpp as5600 example
 */
class As5600 : public MagneticEncoderBase<As5600, as5600_use_high_resolution_timer, 4096,
                                          CONFIG_AS5600_MIN_DIFF> {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = (0b0110110); ///< I2C address of the AS5600

  /// @brief The CRTP base type providing the shared encoder machinery.
  using Base =
      MagneticEncoderBase<As5600, as5600_use_high_resolution_timer, 4096, CONFIG_AS5600_MIN_DIFF>;

  /**
   * @brief Configuration information for the As5600.
   */
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address for this device.
    BasePeripheral<>::write_then_read_fn
        write_then_read;                         ///< Function to write then read from the device.
    velocity_filter_fn velocity_filter{nullptr}; ///< Function to filter the veolcity. @note Will be
                                                 ///< called once every update_period seconds.
    std::chrono::duration<float> update_period{
        .01f}; ///< Update period (1/sample rate) in seconds. This determines the periodicity of the
               ///< timer which will read the position, update the accumulator, and update/filter
               ///< velocity.
    bool auto_init{true}; ///< Whether to automatically initialize the accumulator to the current
                          ///< position on startup.
    bool run_task{true};  ///< Whether to run the timer on startup. If false, you must call update()
                          ///< manually.
    Logger::Verbosity log_level{Logger::Verbosity::WARN};
  };

  /**
   * @brief Construct the As5600 and start the update timer if auto_init and run_task are true.
   * @param config Configuration for the As5600.
   */
  explicit As5600(const Config &config)
      : Base("As5600", config.velocity_filter, config.update_period, config.log_level) {
    set_address(config.device_address);
    set_write_then_read(config.write_then_read);
    if (config.auto_init) {
      std::error_code ec;
      initialize(config.run_task, ec);
    }
  }

  /// @brief Stop the update timer before the object is destroyed.
  ~As5600() { stop(); }

protected:
  // Allow the base to invoke our (protected) read() via CRTP static dispatch.
  friend Base;

  /// @brief Read the current 12-bit angle and update count_.
  /// @param ec Error code to set if there is an error.
  void read(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // read the angle count registers
    uint8_t angle_h = read_u8_from_register((uint8_t)Registers::ANGLE_H, ec);
    if (ec) {
      logger_.error_rate_limited("Error reading: {}", ec.message());
      return;
    }
    uint8_t angle_l = read_u8_from_register((uint8_t)Registers::ANGLE_L, ec);
    if (ec) {
      logger_.error_rate_limited("Error reading: {}", ec.message());
      return;
    }
    // The AS5600 ANGLE is a 12-bit value: ANGLE_H holds Angle[11:8] in its low
    // nibble, ANGLE_L holds Angle[7:0].
    count_ = ((angle_h & 0x0F) << 8) | angle_l;
  }

  /**
   * @brief Register map for the AS5600.
   *
   * @note The AS5600 contains a push-button output (pin 5) with configuration
   *       (mentioned on page 25) via PUSH_THRD register, PUSH_DIFF_DLY
   *       register, and PUSH_TIME_OUT register. However, the register addresses
   *       for these configurations (and their bitfields) are not provided in
   *       the datasheet and must be provided by the manufacturer.
   *
   * @note The push button can only be read from the AS5600 when using SSI
   *       communications, and is returned as part of the magnetic field status
   *       truth table (page 24).
   */
  enum class Registers : uint8_t {
    // configuration registers:
    ZMCO = 0x00,   ///< ZMCO[1:0] (bits 1-0)
    ZPOS_H = 0x01, ///< ZPOS[11:8] (bits 3-0)
    ZPOS_L = 0x02, ///< ZPOS[7:0]
    MPOS_H = 0x03, ///< MPOS[11:8] (bits 3-0)
    MPOS_L = 0x04, ///< MPOS[7:0]
    MANG_H = 0x05, ///< MANG[11:8] (bits 3-0)
    MANG_L = 0x06, ///< MANG[7:0]
    CONF_0 = 0x07, ///< Watchdog (bit 5), Fast Filter Threshold[2:0] (bits 4-2), Slow Filter[1:0]
                   ///< (bits 1-0)
    CONF_1 = 0x08, ///< PWM Freq[1:0] (bits 7-6), Output Stage[1:0] (bits 5-4), HYST[1:0] (bits
                   ///< 3-2), Power Mode[1:0] (bits 1-0)
    // output registers:
    RANG_H = 0x0C,  ///< Raw angle [11:8] (bits 3-0)
    RANG_L = 0x0D,  ///< Raw angle [7:0]
    ANGLE_H = 0x0E, ///< Angle [11:8] (bits 3-0)
    ANGLE_L = 0x0F, ///< Angle [7:0]
    // status registers:
    STATUS = 0x0B, ///< Magnet Detected (bit 5), Magnet too Weak (bit 4), Magnet Too Strong (bit 3)
    AGC = 0x1A,    ///< AGC (automatic gain control)
    MAGN_H = 0x1B, ///< Magnitude[11:8] (bits 3-0)
    MAGN_L = 0x1C, ///< Magnitude[7:0]
    // burn commands:
    BURN = 0xFF, ///< Burn_Angle = 0x80, Burn_Setting = 0x40
  };

  static constexpr int MAGNET_HIGH = (1 << 3);     ///< For use with the STATUS register
  static constexpr int MAGNET_LOW = (1 << 4);      ///< For use with the STATUS register
  static constexpr int MAGNET_DETECTED = (1 << 5); ///< For use with the STATUS register
};
} // namespace espp
