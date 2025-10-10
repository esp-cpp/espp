#pragma once

#include <cstdint>
#include <functional>

#include "base_peripheral.hpp"

namespace espp {
/**
 * LP5817 RGB LED Driver (I2C)
 *
 * Implements a minimal functional interface to control RGB brightness and fades
 * following the register map of the LP5817 datasheet.
 *
 * \note Dot-current (brightness) is controlled on a per-channel basis
 *       using the DC registers. It represents the maximum current
 *       that can be driven through the LED channel, and is an 8-bit value
 *       (0-255). The actual brightness output is then controlled using the
 *       PWM registers, which can be changed on-the-fly to adjust brightness.
 *
 * \note In this driver, brightness is equivalent to the dot-current (DC) setting
 *       in the datasheet, which is an 8-bit value (0-255).
 *
 * \note When initializing, you should set the global max current and the
 *       dot-current (brightness) for each channel to appropriate values
 *       for your application. Then you can use the pwm registers to adjust
 *       brightness on-the-fly, which will also take into account any fading
 *       that is enabled.
 *
 * For more information see the datasheet here:
 * https://www.ti.com/lit/gpn/LP5817
 *
 * \section lp5817_ex1 LP5817 Example
 * \snippet lp5817_example.cpp lp5817 example
 */
class Lp5817 : public BasePeripheral<> {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x2D;   // 7-bit address
  static constexpr uint8_t BROADCAST_ADDRESS = 0x34; // 7-bit address

  /// LED channels
  enum class Channel : uint8_t { OUT0 = 0, OUT1 = 1, OUT2 = 2 };

  /// Global max current settings
  enum class GlobalMaxCurrent : uint8_t {
    MA_25_5 = 0, ///< 25.5mA
    MA_51 = 1    ///< 51mA
  };

  /// Fade time codes (datasheet-defined)
  enum class FadeTime : uint8_t {
    TIME_0 = 0x00,     ///< No fade (instant)
    TIME_50MS = 0x01,  ///< 50ms
    TIME_100MS = 0x02, ///< 100ms
    TIME_150MS = 0x03, ///< 150ms
    TIME_200MS = 0x04, ///< 200ms
    TIME_250MS = 0x05, ///< 250ms
    TIME_300MS = 0x06, ///< 300ms
    TIME_350MS = 0x07, ///< 350ms
    TIME_400MS = 0x08, ///< 400ms
    TIME_450MS = 0x09, ///< 450ms
    TIME_500MS = 0x0A, ///< 500ms
    TIME_1S = 0x0B,    ///< 1 second
    TIME_2S = 0x0C,    ///< 2 seconds
    TIME_4S = 0x0D,    ///< 4 seconds
    TIME_6S = 0x0E,    ///< 6 seconds
    TIME_8S = 0x0F     ///< 8 seconds
  };

  /// Configuration structure
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS;           ///< I2C address of the device
    BasePeripheral::write_fn write;                     ///< Write function
    BasePeripheral::write_then_read_fn write_then_read; ///< Write-then-read function
    bool auto_init = true; ///< If true, initialize() is called in constructor
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log level
  };

  /**
   * @brief Construct a new Lp5817 object
   * @param config Configuration structure
   */
  explicit Lp5817(const Config &config)
      : BasePeripheral({.address = config.device_address,
                        .write = config.write,
                        .write_then_read = config.write_then_read},
                       "Lp5817", config.log_level) {
    if (config.auto_init) {
      std::error_code ec;
      initialize(ec);
      if (ec) {
        logger_.error("Failed to initialize LP5817: {}", ec.message());
      }
    }
  }

  /**
   * @brief Initialize the device.
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   */
  bool initialize(std::error_code &ec) { return init(ec); }

  // Power control
  /**
   * @brief Enable or disable the device.
   * @param on True to enable outputs, false to disable.
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   */
  bool enable(bool on, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("Setting enable to {}", on);
    // set the chip_en bit
    set_bits_in_register_by_mask((uint8_t)Registers::CHIP_EN, CHIP_EN_MASK,
                                 on ? CHIP_EN_MASK : 0x00, ec);
    return !ec;
  }

  /**
   * @brief Check if device is enabled.
   * @param ec Error code set on failure.
   * @return true if device is enabled, false otherwise.
   */
  bool is_enabled(std::error_code &ec) {
    auto reg = read_u8_from_register((uint8_t)Registers::CHIP_EN, ec);
    if (ec)
      return false;
    return (reg & CHIP_EN_MASK) != 0;
  }

  // Per-channel 8-bit brightness (0-255) - DC registers

  /**
   * @brief Set per-channel 8-bit dot-current (DC).
   * @param ch Channel to update.
   * @param value Dot-current [0,255].
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   */
  bool set_dot_current(Channel ch, uint8_t value, std::error_code &ec) {
    logger_.debug("Setting dot-current (DC) of channel {} to {}", ch, value);
    auto reg = dc_register_for(ch);
    write_u8_to_register(reg, value, ec);
    return !ec;
  }

  /**
   * @brief Set per-channel dot-current (DC) using a float value.
   * @param ch Channel to update.
   * @param value Dot-current [0.0, 1.0].
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   * @note Value is clamped to [0.0, 1.0] range.
   */
  bool set_dot_current(Channel ch, float value, std::error_code &ec) {
    value = std::clamp(value, 0.0f, 1.0f);
    return set_dot_current(ch, static_cast<uint8_t>(value * 255.0f), ec);
  }

  /**
   * @brief Set per-channel 8-bit brightness (dot-current).
   * @param ch Channel to update.
   * @param value Dot-Current Brightness [0,255].
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   * @note This is an alias for set_dot_current().
   */
  bool set_brightness(Channel ch, uint8_t value, std::error_code &ec) {
    return set_dot_current(ch, value, ec);
  }

  /**
   * @brief Set per-channel brightness (dot-current) using a float value.
   * @param ch Channel to update.
   * @param value Dot-Current Brightness [0.0, 1.0].
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   * @note Value is clamped to [0.0, 1.0] range.
   */
  bool set_brightness(Channel ch, float value, std::error_code &ec) {
    value = std::clamp(value, 0.0f, 1.0f);
    return set_brightness(ch, static_cast<uint8_t>(value * 255.0f), ec);
  }

  /**
   * @brief Get the current dot-current (DC) of a channel.
   * @param ch Channel to read.
   * @param ec Error code set on failure.
   * @return Dot-current [0,255].
   */
  uint8_t get_dot_current(Channel ch, std::error_code &ec) {
    auto reg = dc_register_for(ch);
    return read_u8_from_register(reg, ec);
  }

  /**
   * @brief Get the brightness (dot-current) of a channel.
   * @param ch Channel to read.
   * @param ec Error code set on failure.
   * @return Dot Current Brightness [0,255].
   */
  uint8_t get_brightness(Channel ch, std::error_code &ec) { return get_dot_current(ch, ec); }

  // Set all channels at once
  /**
   * @brief Set all three channel dot-current (DC) values.
   * @param data Array of three dot-current values [0,255].
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   */
  bool set_dot_current(const std::array<uint8_t, 3> &data, std::error_code &ec) {
    logger_.debug("Setting dot-currents to {}, {}, {}", data[0], data[1], data[2]);
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (!set_dot_current(Channel::OUT0, data[0], ec))
      return false;
    if (!set_dot_current(Channel::OUT1, data[1], ec))
      return false;
    if (!set_dot_current(Channel::OUT2, data[2], ec))
      return false;
  }

  /**
   * @brief Set all three channel dot-current (DC) values.
   * @param r Red dot-current [0,255].
   * @param g Green dot-current [0,255].
   * @param b Blue dot-current [0,255].
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   * @note This assumes RGB are mapped to channels 0, 1, 2 respectively.
   */
  bool set_rgb_dot_current(uint8_t r, uint8_t g, uint8_t b, std::error_code &ec) {
    return set_dot_current({r, g, b}, ec);
  }

  /**
   * @brief Set all three channel brightness (dot-current) values.
   * @param r Red Dot-Current brightness [0,255].
   * @param g Green Dot-Current brightness [0,255].
   * @param b Blue Dot-Current brightness [0,255].
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   * @note This assumes RGB are mapped to channels 0, 1, 2 respectively.
   * @note This is an alias for set_rgb_dot_current().
   */
  bool set_rgb(uint8_t r, uint8_t g, uint8_t b, std::error_code &ec) {
    return set_rgb_dot_current(r, g, b, ec);
  }

  /**
   * @brief Set all three channel brightness (dot-current) values using float values.
   * @param r Red Dot-Current brightness [0.0, 1.0].
   * @param g Green Dot-Current brightness [0.0, 1.0].
   * @param b Blue Dot-Current brightness [0.0, 1.0].
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   * @note Values are clamped to [0.0, 1.0] range.
   * @note This assumes RGB are mapped to channels 0, 1, 2 respectively.
   * @note This is an alias for set_rgb_dot_current().
   */
  bool set_rgb(float r, float g, float b, std::error_code &ec) {
    r = std::clamp(r, 0.0f, 1.0f);
    g = std::clamp(g, 0.0f, 1.0f);
    b = std::clamp(b, 0.0f, 1.0f);
    return set_rgb(static_cast<uint8_t>(r * 255.0f), static_cast<uint8_t>(g * 255.0f),
                   static_cast<uint8_t>(b * 255.0f), ec);
  }

  /**
   * @brief Set all three channel manual PWM values (manual mode).
   * @param data Array of three manual PWM values [0,255].
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   */
  bool set_pwm(const std::array<uint8_t, 3> &data, std::error_code &ec) {
    logger_.debug("Setting PWM to {}, {}, {}", data[0], data[1], data[2]);
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (!set_manual_pwm(Channel::OUT0, data[0], ec))
      return false;
    if (!set_manual_pwm(Channel::OUT1, data[1], ec))
      return false;
    if (!set_manual_pwm(Channel::OUT2, data[2], ec))
      return false;
    return true;
  }

  /**
   * @brief Set all three channel manual PWM values (manual mode).
   * @param r Red manual PWM value [0,255].
   * @param g Green manual PWM value [0,255].
   * @param b Blue manual PWM value [0,255].
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   * @note This assumes RGB are mapped to channels 0, 1, 2 respectively.
   * @note This is best used when fading is enabled on all channels and there
   *       is a non-zero fade time, to avoid visible stepping.
   */
  bool set_rgb_pwm(uint8_t r, uint8_t g, uint8_t b, std::error_code &ec) {
    logger_.debug("Setting RGB manual PWM to ({}, {}, {})", r, g, b);
    return set_pwm({r, g, b}, ec);
  }

  /**
   * @brief Set all three channel manual PWM values using float values (manual mode).
   * @param r Red manual PWM value [0.0, 1.0].
   * @param g Green manual PWM value [0.0, 1.0].
   * @param b Blue manual PWM value [0.0, 1.0].
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   * @note Values are clamped to [0.0, 1.0] range.
   * @note This assumes RGB are mapped to channels 0, 1, 2 respectively.
   * @note This is best used when fading is enabled on all channels and there
   *       is a non-zero fade time configured.
   */
  bool set_rgb_pwm(float r, float g, float b, std::error_code &ec) {
    r = std::clamp(r, 0.0f, 1.0f);
    g = std::clamp(g, 0.0f, 1.0f);
    b = std::clamp(b, 0.0f, 1.0f);
    return set_rgb_pwm(static_cast<uint8_t>(r * 255.0f), static_cast<uint8_t>(g * 255.0f),
                       static_cast<uint8_t>(b * 255.0f), ec);
  }

  /**
   * @brief Set manual PWM value for a channel (manual mode).
   * @param ch Channel to update.
   * @param value Manual PWM value [0,255].
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   * @note This is best used when fading is enabled on the channel and
   *       there is a non-zero fade time configured.
   */
  bool set_manual_pwm(Channel ch, uint8_t value, std::error_code &ec) {
    logger_.debug("Setting manual PWM of channel {} to {}", ch, value);
    auto reg = manual_pwm_register_for(ch);
    write_u8_to_register(reg, value, ec);
    return !ec;
  }

  /**
   * @brief Set manual PWM value for a channel using a float value (manual mode).
   * @param ch Channel to update.
   * @param value Manual PWM value [0.0, 1.0].
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   * @note Value is clamped to [0.0, 1.0] range.
   */
  bool set_manual_pwm(Channel ch, float value, std::error_code &ec) {
    value = std::clamp(value, 0.0f, 1.0f);
    return set_manual_pwm(ch, static_cast<uint8_t>(value * 255.0f), ec);
  }

  /**
   * @brief Get the current manual PWM value of a channel (manual mode).
   * @param ch Channel to read.
   * @param ec Error code set on failure.
   * @return Manual PWM value [0,255].
   */
  uint8_t get_manual_pwm(Channel ch, std::error_code &ec) {
    auto reg = manual_pwm_register_for(ch);
    return read_u8_from_register(reg, ec);
  }

  /**
   * @brief Set global fade time (datasheet-defined).
   * @param time Fade time.
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   * @note This sets the fade time for all channels.
   * @note You must call update() after changing this setting for it to take
   *       effect.
   */
  bool set_fade_time(FadeTime time, std::error_code &ec) {
    logger_.debug("Setting fade time to {}", time);
    return set_fade_time_code(static_cast<uint8_t>(time), ec);
  }

  /**
   * @brief Get the current global fade time setting.
   * @param ec Error code set on failure.
   * @return Current global fade time setting.
   */
  FadeTime get_fade_time(std::error_code &ec) {
    auto code = get_fade_time_code(ec);
    if (ec)
      return FadeTime::TIME_0; // default
    return static_cast<FadeTime>(code);
  }

  /**
   * @brief Enable/disable fade engine for a channel.
   * @param ch Channel to configure.
   * @param enable True to enable fading, false to disable.
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   * @note Fading must be enabled for exponential dimming to have any effect.
   * @note You must call update() after changing this setting for it to take
   *       effect.
   */
  bool set_fade_enable(Channel ch, bool enable, std::error_code &ec) {
    logger_.debug("Setting fade enable of channel {} to {}", ch, enable);
    uint8_t bit = 1 << (uint8_t)ch; // bits 0..2
    set_bits_in_register_by_mask((uint8_t)Registers::DEV_CONFIG2, bit, enable ? bit : 0, ec);
    return !ec;
  }

  /**
   * @brief Is fading enabled for a channel?
   * @param ch Channel to check.
   * @param ec Error code set on failure.
   * @return true if fading is enabled, false otherwise.
   */
  bool is_fade_enabled(Channel ch, std::error_code &ec) {
    auto reg = read_u8_from_register((uint8_t)Registers::DEV_CONFIG2, ec);
    if (ec)
      return false;
    uint8_t bit = 1 << (uint8_t)ch; // bits 0..2
    return (reg & bit) != 0;
  }

  /**
   * @brief Enable/disable exponential dimming for a channel.
   * @param ch Channel to configure.
   * @param enable True to enable exponential dimming, false for linear.
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   * @note Exponential dimming is only effective if fading is also enabled.
   *       See datasheet for details.
   * @note You must call update() after changing this setting for it to take
   *       effect.
   */
  bool set_exponential_dimming_enable(Channel ch, bool enable, std::error_code &ec) {
    uint8_t bit = 1 << (4 + (uint8_t)ch); // bits 4..6
    set_bits_in_register_by_mask((uint8_t)Registers::DEV_CONFIG3, bit, enable ? bit : 0, ec);
    return !ec;
  }

  /**
   * @brief Is exponential dimming enabled for a channel?
   * @param ch Channel to check.
   * @param ec Error code set on failure.
   * @return true if exponential dimming is enabled, false otherwise.
   */
  bool is_exponential_dimming_enabled(Channel ch, std::error_code &ec) {
    auto reg = read_u8_from_register((uint8_t)Registers::DEV_CONFIG3, ec);
    if (ec)
      return false;
    uint8_t bit = 1 << (4 + (uint8_t)ch);
    return (reg & bit) != 0;
  }

  /**
   * @brief Enable/disable output stage for a channel.
   * @param ch Channel to configure.
   * @param enable True to enable output, false to disable.
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   * @note You must call update() after changing this setting for it to take
   *       effect.
   */
  bool set_output_enable(Channel ch, bool enable, std::error_code &ec) {
    uint8_t bit = 1 << (uint8_t)ch; // bits 0..2
    set_bits_in_register_by_mask((uint8_t)Registers::DEV_CONFIG1, bit, enable ? bit : 0, ec);
    return !ec;
  }

  /**
   * @brief Check if output stage is enabled for a channel.
   * @param ch Channel to check.
   * @param ec Error code set on failure.
   * @return true if output stage is enabled, false otherwise.
   */
  bool is_output_enabled(Channel ch, std::error_code &ec) {
    auto reg = read_u8_from_register((uint8_t)Registers::DEV_CONFIG1, ec);
    if (ec)
      return false;
    uint8_t bit = 1 << (uint8_t)ch; // bits 0..2
    return (reg & bit) != 0;
  }

  /**
   * @brief Set global max current.
   * @param current Global max current setting.
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   * @note This sets the max current for all channels.
   * @note You must call update() after changing this setting for it to take
   *       effect.
   */
  bool set_max_current(GlobalMaxCurrent current, std::error_code &ec) {
    set_max_current_code(static_cast<uint8_t>(current), ec);
    return !ec;
  }

  /**
   * @brief Get the current global max current setting.
   * @param ec Error code set on failure.
   * @return Current global max current setting.
   */
  GlobalMaxCurrent get_max_current(std::error_code &ec) {
    auto code = get_max_current_code(ec);
    if (ec)
      return GlobalMaxCurrent::MA_25_5; // default
    return static_cast<GlobalMaxCurrent>(code);
  }

  /**
   * @brief Issue update command (latch new settings).
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   */
  bool update(std::error_code &ec) {
    write_u8_to_register((uint8_t)Registers::UPDATE_CMD, 0x55, ec);
    return !ec;
  }

  /**
   * @brief Issue reset command.
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   */
  bool reset(std::error_code &ec) {
    write_u8_to_register((uint8_t)Registers::RESET_CMD, 0xCC, ec);
    return !ec;
  }

  /**
   * @brief Issue shutdown command.
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   */
  bool shutdown(std::error_code &ec) {
    write_u8_to_register((uint8_t)Registers::SHUTDOWN_CMD, 0x33, ec);
    return !ec;
  }

  /**
   * @brief Read flag register (POR/TSD).
   * @param ec Error code set on failure.
   * @return Flag register value.
   * @note Bit0 = POR (Power-On Reset), Bit1 = TSD (Thermal Shutdown)
   */
  uint8_t read_flags(std::error_code &ec) {
    return read_u8_from_register((uint8_t)Registers::FLAG, ec);
  }

  /**
   * @brief Check if POR (Power-On Reset) flag is set.
   * @param ec Error code set on failure.
   * @return true if POR flag is set, false otherwise.
   */
  bool is_por_flag_set(std::error_code &ec) {
    auto flags = read_flags(ec);
    if (ec)
      return false;
    return (flags & FLAG_CLR_POR_MASK) != 0;
  }

  /**
   * @brief Check if TSD (Thermal Shutdown) flag is set.
   * @param ec Error code set on failure.
   * @return true if TSD flag is set, false otherwise.
   */
  bool is_tsd_flag_set(std::error_code &ec) {
    auto flags = read_flags(ec);
    if (ec)
      return false;
    return (flags & FLAG_CLR_TSD_MASK) != 0;
  }

  /**
   * @brief Clear both POR (Power-On Reset) and TSD (Thermal Shutdown) flags.
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   */
  bool clear_flags(std::error_code &ec) {
    write_u8_to_register((uint8_t)Registers::FLAG_CLR, FLAG_CLR_POR_MASK | FLAG_CLR_TSD_MASK, ec);
    return !ec;
  }

  /**
   * @brief Clear POR (Power-On Reset) flag.
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   */
  bool clear_por_flag(std::error_code &ec) {
    set_bits_in_register_by_mask((uint8_t)Registers::FLAG_CLR, FLAG_CLR_POR_MASK, FLAG_CLR_POR_MASK,
                                 ec);
    return !ec;
  }

  /**
   * @brief Clear TSD (Thermal Shutdown) flag.
   * @param ec Error code set on failure.
   * @return true on success, false on failure.
   */
  bool clear_tsd_flag(std::error_code &ec) {
    set_bits_in_register_by_mask((uint8_t)Registers::FLAG_CLR, FLAG_CLR_TSD_MASK, FLAG_CLR_TSD_MASK,
                                 ec);
    return !ec;
  }

protected:
  // register map (from TI LP5817)
  enum class Registers : uint8_t {
    CHIP_EN = 0x00,     // bit0 CHIP_EN
    DEV_CONFIG0 = 0x01, // bit0 MAX_CURRENT
    DEV_CONFIG1 = 0x02, // bits[2:0] OUTx_EN
    DEV_CONFIG2 = 0x03, // bits[2:0] OUTx_FADE_EN, bits[7:4] LED_FADE_TIME
    DEV_CONFIG3 = 0x04, // bits[6:4] OUTx_EXP_EN
    // 0x05..0x0C reserved
    SHUTDOWN_CMD = 0x0D,
    RESET_CMD = 0x0E,
    UPDATE_CMD = 0x0F,
    // 0x10..0x12 reserved
    FLAG_CLR = 0x13, // bit0 POR_CLR, bit1 TSD_CLR
    OUT0_DC = 0x14,
    OUT1_DC = 0x15,
    OUT2_DC = 0x16,
    // 0x17 reserved
    OUT0_MANUAL_PWM = 0x18,
    OUT1_MANUAL_PWM = 0x19,
    OUT2_MANUAL_PWM = 0x1A,
    // 0x1B..0x3F reserved
    FLAG = 0x40 // bit0 POR, bit1 TSD
  };

  inline uint8_t dc_register_for(Channel ch) const {
    switch (ch) {
    case Channel::OUT0:
      return (uint8_t)Registers::OUT0_DC;
    case Channel::OUT1:
      return (uint8_t)Registers::OUT1_DC;
    case Channel::OUT2:
      return (uint8_t)Registers::OUT2_DC;
    default:
      return (uint8_t)Registers::OUT0_DC;
    }
  }

  inline uint8_t manual_pwm_register_for(Channel ch) const {
    switch (ch) {
    case Channel::OUT0:
      return (uint8_t)Registers::OUT0_MANUAL_PWM;
    case Channel::OUT1:
      return (uint8_t)Registers::OUT1_MANUAL_PWM;
    case Channel::OUT2:
      return (uint8_t)Registers::OUT2_MANUAL_PWM;
    default:
      return (uint8_t)Registers::OUT0_MANUAL_PWM;
    }
  }

  bool init(std::error_code &ec) {
    if (!enable(true, ec)) {
      logger_.error("Failed to enable device: {}", ec.message());
      return false;
    }
    // reset device to defaults
    return reset(ec);
  }

  bool set_fade_time_code(uint8_t code, std::error_code &ec) {
    uint8_t value = (uint8_t)((code & 0x0F) << 4); // bits [7:4]
    set_bits_in_register_by_mask((uint8_t)Registers::DEV_CONFIG2, DEV_CONFIG2_FADE_TIME_MASK, value,
                                 ec);
    return !ec;
  }

  uint8_t get_fade_time_code(std::error_code &ec) {
    auto reg = read_u8_from_register((uint8_t)Registers::DEV_CONFIG2, ec);
    if (ec)
      return 0; // default
    return (reg & DEV_CONFIG2_FADE_TIME_MASK) >> 4;
  }

  bool set_max_current_code(uint8_t code, std::error_code &ec) {
    uint8_t value = (uint8_t)(code & 0x01); // bit0 only
    set_bits_in_register_by_mask((uint8_t)Registers::DEV_CONFIG0, MAX_CURRENT_MASK, value, ec);
    return !ec;
  }

  uint8_t get_max_current_code(std::error_code &ec) {
    auto reg = read_u8_from_register((uint8_t)Registers::DEV_CONFIG0, ec);
    if (ec)
      return 0; // default
    return reg & MAX_CURRENT_MASK;
  }

  // Bit masks per datasheet
  static constexpr uint8_t CHIP_EN_MASK = 0x01;               // CHIP_EN bit
  static constexpr uint8_t MAX_CURRENT_MASK = 0x01;           // DEV_CONFIG0 bit0
  static constexpr uint8_t DEV_CONFIG2_FADE_TIME_MASK = 0xF0; // DEV_CONFIG2 bits[7:4]
  static constexpr uint8_t FLAG_CLR_POR_MASK = 0x01;          // bit0
  static constexpr uint8_t FLAG_CLR_TSD_MASK = 0x02;          // bit1
};
} // namespace espp

// for libfmt formatting of enums
template <> struct fmt::formatter<espp::Lp5817::Channel> : fmt::formatter<std::string_view> {
  template <typename FormatContext> auto format(espp::Lp5817::Channel c, FormatContext &ctx) const {
    std::string_view name = "UNKNOWN";
    switch (c) {
    case espp::Lp5817::Channel::OUT0:
      name = "OUT0";
      break;
    case espp::Lp5817::Channel::OUT1:
      name = "OUT1";
      break;
    case espp::Lp5817::Channel::OUT2:
      name = "OUT2";
      break;
    }
    return fmt::formatter<std::string_view>::format(name, ctx);
  }
};

template <>
struct fmt::formatter<espp::Lp5817::GlobalMaxCurrent> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(espp::Lp5817::GlobalMaxCurrent c, FormatContext &ctx) const {
    std::string_view name = "UNKNOWN";
    switch (c) {
    case espp::Lp5817::GlobalMaxCurrent::MA_25_5:
      name = "25.5mA";
      break;
    case espp::Lp5817::GlobalMaxCurrent::MA_51:
      name = "51mA";
      break;
    }
    return fmt::formatter<std::string_view>::format(name, ctx);
  }
};

template <> struct fmt::formatter<espp::Lp5817::FadeTime> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(espp::Lp5817::FadeTime c, FormatContext &ctx) const {
    std::string_view name = "UNKNOWN";
    switch (c) {
    case espp::Lp5817::FadeTime::TIME_0:
      name = "0ms";
      break;
    case espp::Lp5817::FadeTime::TIME_50MS:
      name = "50ms";
      break;
    case espp::Lp5817::FadeTime::TIME_100MS:
      name = "100ms";
      break;
    case espp::Lp5817::FadeTime::TIME_150MS:
      name = "150ms";
      break;
    case espp::Lp5817::FadeTime::TIME_200MS:
      name = "200ms";
      break;
    case espp::Lp5817::FadeTime::TIME_250MS:
      name = "250ms";
      break;
    case espp::Lp5817::FadeTime::TIME_300MS:
      name = "300ms";
      break;
    case espp::Lp5817::FadeTime::TIME_350MS:
      name = "350ms";
      break;
    case espp::Lp5817::FadeTime::TIME_400MS:
      name = "400ms";
      break;
    case espp::Lp5817::FadeTime::TIME_450MS:
      name = "450ms";
      break;
    case espp::Lp5817::FadeTime::TIME_500MS:
      name = "500ms";
      break;
    case espp::Lp5817::FadeTime::TIME_1S:
      name = "1s";
      break;
    case espp::Lp5817::FadeTime::TIME_2S:
      name = "2s";
      break;
    case espp::Lp5817::FadeTime::TIME_4S:
      name = "4s";
      break;
    case espp::Lp5817::FadeTime::TIME_6S:
      name = "6s";
      break;
    case espp::Lp5817::FadeTime::TIME_8S:
      name = "8s";
      break;
    }
    return fmt::formatter<std::string_view>::format(name, ctx);
  }
};
