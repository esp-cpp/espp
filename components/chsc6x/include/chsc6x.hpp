#pragma once

#include <functional>

#include "base_peripheral.hpp"
#include "touch.hpp"

namespace espp {
/// @brief Driver for the Chsc6x touch controller
///
/// \section chsc6x_ex1 Example
/// \snippet chsc6x_example.cpp chsc6x example
class Chsc6x : public BasePeripheral<>, public ITouchDevice {
public:
  /// Default address for the CHSC6X chip
  static constexpr uint8_t DEFAULT_ADDRESS = 0x2E;

  /// @brief Configuration for the CHSC6X driver
  struct Config {
    BasePeripheral::write_fn write;    ///< Function for writing to the CHSC6X chip
    BasePeripheral::read_fn read;      ///< Function for reading from the CHSC6X chip
    uint8_t address = DEFAULT_ADDRESS; ///< Which address to use for this chip?
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; ///< Log verbosity for the input driver.
  };

  /// @brief Constructor for the CHSC6X driver
  /// @param config The configuration for the driver
  explicit Chsc6x(const Config &config)
      : BasePeripheral({.address = config.address, .write = config.write, .read = config.read},
                       "Chsc6x", config.log_level) {}

  /// @brief Update the state of the CHSC6X driver
  /// @param ec Error code to set if an error occurs
  /// @return True if the CHSC6X has new data, false otherwise
  bool update(std::error_code &ec) {
    TouchState state{};
    static constexpr size_t DATA_LEN = 5;
    static uint8_t data[DATA_LEN];
    read_many_from_register(0, data, DATA_LEN, ec);
    if (ec)
      return false;

    // first byte is non-zero when touched, 3rd byte is x, 5th byte is y
    if (data[0] == 0) {
      std::lock_guard<std::recursive_mutex> lock(base_mutex_);
      touch_state_ = state;
      return true;
    }
    state.num_touch_points = 1;
    state.points[0] = {.x = data[2], .y = data[4]};
    logger_.debug("Touch at ({}, {})", state.points[0].x, state.points[0].y);
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    touch_state_ = state;
    return true;
  }

  /// @brief Get the cached touch state.
  /// @return The cached touch state as of the last update() call.
  TouchState touch_state() const override {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    return touch_state_;
  }

  /// @brief Get the number of touch points
  /// @return The number of touch points as of the last update
  /// @note This is a cached value from the last update() call
  uint8_t get_num_touch_points() const { return touch_state().num_touch_points; }

  /// @brief Get the touch point data
  /// @param num_touch_points The number of touch points as of the last update
  /// @param x The x coordinate of the touch point
  /// @param y The y coordinate of the touch point
  /// @note This is a cached value from the last update() call
  void get_touch_point(uint8_t *num_touch_points, uint16_t *x, uint16_t *y) const {
    auto state = touch_state();
    auto point = state.primary_point();
    *num_touch_points = state.num_touch_points;
    *x = point.x;
    *y = point.y;
  }

protected:
  TouchState touch_state_;
}; // class Chsc6x
} // namespace espp
