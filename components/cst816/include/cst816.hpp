#pragma once

#include <atomic>
#include <functional>

#include "base_peripheral.hpp"

namespace espp {
/// @brief Driver for the CST816 touch controller
/// @note This chip does not respond to I2C commands normally, only after an
///       event. To properly interact with this chip, you should register an
///       interrupt on the chip's IRQ line. After the IRQ line is asserted, it
///       will respond to I2C reads for a short period of time.
///
/// For more information, you can look at some reference code and the datasheet
/// here:
/// https://github.com/espressif/esp-bsp/tree/master/components/lcd_touch/esp_lcd_touch_cst816s
///
/// \section cst816_ex1 Example
/// \snippet cst816_example.cpp cst816 example
class Cst816 : public BasePeripheral<std::uint8_t> {
public:
  /// Default address for the CST816 chip
  static constexpr uint8_t DEFAULT_ADDRESS = 0x15;

  /// @brief Configuration for the CST816 driver
  struct Config {
    BasePeripheral::write_fn write;    ///< Function for writing to the CST816 chip
    BasePeripheral::read_fn read;      ///< Function for reading from the CST816 chip
    uint8_t address = DEFAULT_ADDRESS; ///< Which address to use for this chip?
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; ///< Log verbosity for the input driver.
  };

  /// @brief Constructor for the CST816 driver
  /// @param config The configuration for the driver
  explicit Cst816(const Config &config)
      : BasePeripheral({.address = config.address, .write = config.write, .read = config.read},
                       "Cst816", config.log_level) {}

  /// @brief Update the state of the CST816 driver
  /// @param ec Error code to set if an error occurs
  /// @return True if the CST816 has new data, false otherwise
  bool update(std::error_code &ec) {
    bool new_data = false;
    Data data{};
    read_many_from_register((uint8_t)Registers::DATA_START, (uint8_t *)&data, sizeof(data), ec);
    if (ec)
      return false;

    num_touch_points_ = data.num;
    x_ = (data.x_h << 8) | data.x_l;
    y_ = (data.y_h << 8) | data.y_l;
    home_button_pressed_ = false;
    new_data = true;
    return new_data;
  }

  /// @brief Get the number of touch points
  /// @return The number of touch points as of the last update
  /// @note This is a cached value from the last update() call
  uint8_t get_num_touch_points() const { return num_touch_points_; }

  /// @brief Get the touch point data
  /// @param num_touch_points The number of touch points as of the last update
  /// @param x The x coordinate of the touch point
  /// @param y The y coordinate of the touch point
  /// @note This is a cached value from the last update() call
  void get_touch_point(uint8_t *num_touch_points, uint16_t *x, uint16_t *y) const {
    *num_touch_points = get_num_touch_points();
    if (*num_touch_points != 0) {
      *x = x_;
      *y = y_;
    }
  }

  /// @brief Get the home button state
  /// @return True if the home button is pressed, false otherwise
  /// @note This is a cached value from the last update() call
  bool get_home_button_state() const { return home_button_pressed_; }

protected:
  static constexpr int MAX_CONTACTS = 1;

  struct Data {
    uint8_t num = 0;
    uint8_t x_h : 4;
    uint8_t : 4;
    uint8_t x_l = 0;
    uint8_t y_h : 4;
    uint8_t : 4;
    uint8_t y_l = 0;
  };

  enum class Registers : uint8_t {
    DATA_START = 0x02,
    CHIP_ID = 0xA7,
  };

  std::atomic<bool> home_button_pressed_{false};
  std::atomic<uint8_t> num_touch_points_;
  std::atomic<uint16_t> x_;
  std::atomic<uint16_t> y_;
};
} // namespace espp
