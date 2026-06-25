#pragma once

#include <atomic>
#include <system_error>

#include "base_peripheral.hpp"
#include "touch.hpp"

namespace espp {

/// @brief Driver for the ST7123 integrated touch controller
///
/// The ST7123 is a TDDI (Touch and Display Driver Integration) chip that
/// includes both a MIPI-DSI display driver and a capacitive multi-touch
/// controller. The touch data is accessed via I2C at the chip's address
/// (default 0x55).
///
/// @note The ST7123's touch engine is gated by the LCD reset (LCD_RST) line,
///       NOT the TP_RST line used by standalone touch controllers such as the
///       GT911. When used in a system that has a separate TP_RST signal
///       (e.g. M5Stack Tab5), do NOT toggle TP_RST for this chip — doing so
///       may knock the touch I2C endpoint offline.
///
/// Touch data reading sequence (based on ST7123 TDDI Interface Protocol):
///  1. Read 1 byte from register 0x0010 (advanced info). Bit 3 = with_coord.
///  2. If with_coord is set:
///     a. Read 1 byte from register 0x0009 (max touch count).
///     b. Read (max_touches × 7) bytes from register 0x0014 (touch reports).
///     c. For each 7-byte report: bit 7 of byte[0] = valid flag,
///        x = ((byte[0] & 0x3F) << 8) | byte[1],
///        y = (byte[2] << 8) | byte[3].
///
/// \section st7123touch_ex1 Example
/// \snippet st7123touch_example.cpp st7123touch example
class St7123Touch : public BasePeripheral<std::uint16_t> {
public:
  /// Default I2C address for the ST7123 touch interface
  static constexpr uint8_t DEFAULT_ADDRESS = 0x55;

  /// @brief Configuration for the St7123Touch driver
  /// @note The ST7123 latches its register pointer only for the duration of a
  ///       single I2C transaction. Register reads MUST therefore use a combined
  ///       repeated-START write-then-read; issuing the register-pointer write
  ///       and the data read as two separate transactions (with a STOP in
  ///       between) makes the chip return register 0x00 on every read. For that
  ///       reason the driver takes a `write_then_read` function rather than
  ///       separate write/read functions.
  struct Config {
    BasePeripheral::write_then_read_fn
        write_then_read;               ///< Combined (repeated-START) write-then-read for the ST7123
    uint8_t address = DEFAULT_ADDRESS; ///< I2C address of the chip
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; ///< Log verbosity for the driver
  };

  /// @brief Constructor for the St7123Touch driver
  /// @param config The configuration for the driver
  explicit St7123Touch(const Config &config)
      : BasePeripheral({.address = config.address, .write_then_read = config.write_then_read},
                       "St7123Touch", config.log_level) {}

  /// @brief Update the touch state by reading from the ST7123 over I2C
  /// @param ec Error code to set if an I2C error occurs
  /// @return True when the read succeeded (regardless of whether a finger is
  ///         actually touching), false on I2C error
  bool update(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // Read advanced info byte: bit 3 (with_coord) indicates coordinate data
    uint8_t adv_info = 0;
    read_many_from_register(static_cast<uint16_t>(Registers::ADV_INFO), &adv_info, 1, ec);
    if (ec)
      return false;

    if (!(adv_info & ADV_INFO_WITH_COORD)) {
      // No coordinate data in this interrupt — clear touch state so LVGL sees
      // the finger as lifted.
      num_touch_points_ = 0;
      x_ = 0;
      y_ = 0;
      return true;
    }

    // Read the firmware-configured maximum number of simultaneous touches
    uint8_t max_touches = 0;
    read_many_from_register(static_cast<uint16_t>(Registers::MAX_TOUCHES), &max_touches, 1, ec);
    if (ec)
      return false;

    if (max_touches == 0) {
      num_touch_points_ = 0;
      x_ = 0;
      y_ = 0;
      return true;
    }

    // Clamp to our local buffer size
    if (max_touches > MAX_TOUCH_POINTS) {
      max_touches = MAX_TOUCH_POINTS;
    }

    // Read all touch reports in one transaction
    uint8_t data[MAX_TOUCH_POINTS * TOUCH_REPORT_SIZE] = {};
    read_many_from_register(static_cast<uint16_t>(Registers::REPORT_COORD_0), data,
                            TOUCH_REPORT_SIZE * max_touches, ec);
    if (ec)
      return false;

    // Parse reports; record first valid point for single-touch consumers
    uint8_t count = 0;
    uint16_t first_x = 0;
    uint16_t first_y = 0;
    for (int i = 0; i < max_touches; i++) {
      const uint8_t *p = &data[i * TOUCH_REPORT_SIZE];
      const bool valid = (p[0] & 0x80) != 0;
      if (!valid)
        continue;
      const uint16_t px = static_cast<uint16_t>((p[0] & 0x3F) << 8) | p[1];
      const uint16_t py = static_cast<uint16_t>(p[2] << 8) | p[3];
      if (count == 0) {
        first_x = px;
        first_y = py;
      }
      count++;
    }

    num_touch_points_ = count;
    x_ = first_x;
    y_ = first_y;
    logger_.debug("Touch: {} point(s) at ({}, {})", count, first_x, first_y);
    return true;
  }

  /// @brief Get the number of active touch points
  /// @return Touch point count as of the last update() call
  uint8_t get_num_touch_points() const { return num_touch_points_; }

  /// @brief Get the primary touch point coordinates
  /// @param num_touch_points Output: number of active touch points
  /// @param x Output: X coordinate of the first active touch point
  /// @param y Output: Y coordinate of the first active touch point
  /// @note The values are cached from the last update() call.
  void get_touch_point(uint8_t *num_touch_points, uint16_t *x, uint16_t *y) const {
    *num_touch_points = get_num_touch_points();
    if (*num_touch_points != 0) {
      *x = x_;
      *y = y_;
    }
  }

  /// @brief Get the home-button state
  /// @return Always false — the ST7123 does not expose a home button via I2C
  bool get_home_button_state() const { return false; }

protected:
  /// Maximum number of simultaneous touches the driver will parse
  static constexpr int MAX_TOUCH_POINTS = 10;
  /// Bytes per touch report in the coordinate register block
  static constexpr int TOUCH_REPORT_SIZE = 7;
  /// Bit mask for the with_coord flag in the advanced-info register
  static constexpr uint8_t ADV_INFO_WITH_COORD = (1 << 3);

  enum class Registers : uint16_t {
    ADV_INFO = 0x0010,       ///< Advanced info byte; bit 3 = with_coord
    MAX_TOUCHES = 0x0009,    ///< Firmware-configured maximum touch count
    REPORT_COORD_0 = 0x0014, ///< First touch coordinate report (7 bytes each)
  };

  std::atomic<uint8_t> num_touch_points_{0};
  std::atomic<uint16_t> x_{0};
  std::atomic<uint16_t> y_{0};
}; // class St7123Touch
} // namespace espp
