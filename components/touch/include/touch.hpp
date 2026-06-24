#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <system_error>

namespace espp {
/// The data structure for a single touch point.
struct TouchPoint {
  uint16_t x = 0; ///< The x coordinate.
  uint16_t y = 0; ///< The y coordinate.

  /// @brief Compare two TouchPoint objects for equality.
  /// @param rhs The right hand side of the comparison.
  /// @return true if the two TouchPoint objects are equal, false otherwise.
  bool operator==(const TouchPoint &rhs) const = default;
};

/// The data structure for the primary touch point used by single-pointer consumers.
struct TouchpadData {
  uint8_t num_touch_points = 0; ///< The number of touch points.
  uint16_t x = 0;               ///< The primary touch x coordinate.
  uint16_t y = 0;               ///< The primary touch y coordinate.
  uint8_t btn_state = 0;        ///< The button state (0 = released, 1 = pressed).

  /// @brief Compare two TouchpadData objects for equality.
  /// @param rhs The right hand side of the comparison.
  /// @return true if the two TouchpadData objects are equal, false otherwise.
  bool operator==(const TouchpadData &rhs) const = default;
};

/// Shared cached touch state used by touch drivers.
struct TouchState {
  static constexpr std::size_t MAX_TOUCH_POINTS = 5; ///< Maximum number of cached touch points.

  std::array<TouchPoint, MAX_TOUCH_POINTS> points{}; ///< Cached touch points.
  uint8_t num_touch_points = 0;                      ///< Number of valid touch points.
  uint8_t btn_state = 0;                             ///< Optional button state.

  /// @brief Compare two TouchState objects for equality.
  /// @param rhs The right hand side of the comparison.
  /// @return true if the two TouchState objects are equal, false otherwise.
  bool operator==(const TouchState &rhs) const = default;

  /// @brief Get the primary touch point.
  /// @return The first valid touch point, or `{0, 0}` if there is no touch.
  TouchPoint primary_point() const {
    if (num_touch_points == 0) {
      return {};
    }
    return points[0];
  }

  /// @brief Convert the cached multi-touch state to the single-point touchpad view.
  /// @return The primary-point view of the cached state.
  TouchpadData touchpad_data() const {
    auto point = primary_point();
    return {
        .num_touch_points = num_touch_points, .x = point.x, .y = point.y, .btn_state = btn_state};
  }
};

/// Runtime interface implemented by touch controller drivers.
class ITouchDevice {
public:
  virtual ~ITouchDevice() = default;

  /// @brief Update the cached touch state from the hardware.
  /// @param ec Error code to set if an error occurs.
  /// @return True if the device reported new data, false otherwise.
  virtual bool update(std::error_code &ec) = 0;

  /// @brief Get the cached touch state.
  /// @return The cached touch state.
  virtual TouchState touch_state() const = 0;

  /// @brief Whether the controller exposes a touch-adjacent home button.
  /// @return True if the controller exposes a home button, false otherwise.
  virtual bool has_home_button() const { return false; }

  /// @brief Get the number of cached touch points.
  /// @return The number of cached touch points.
  uint8_t get_num_touch_points() const { return touch_state().num_touch_points; }

  /// @brief Get the primary cached touch point.
  /// @param num_touch_points The number of touch points as of the last update.
  /// @param x The x coordinate of the primary touch point.
  /// @param y The y coordinate of the primary touch point.
  void get_touch_point(uint8_t *num_touch_points, uint16_t *x, uint16_t *y) const {
    auto state = touch_state();
    auto point = state.primary_point();
    *num_touch_points = state.num_touch_points;
    *x = point.x;
    *y = point.y;
  }

  /// @brief Get the cached home button state.
  /// @return The cached home button state.
  uint8_t get_home_button_state() const { return touch_state().btn_state; }

  /// @brief Get the cached single-point touchpad view.
  /// @return The primary touchpad view of the cached state.
  TouchpadData touchpad_data() const { return touch_state().touchpad_data(); }
};

/// @brief Concept satisfied by any touch-controller driver that exposes the
///        standard espp touch-driver interface.
///
/// A type T satisfies TouchDriverConcept if it provides:
/// - `bool T::update(std::error_code &)` — read new touch data from hardware
/// - `void T::get_touch_point(uint8_t *, uint16_t *, uint16_t *) const` — retrieve coordinates
/// - `bool T::get_home_button_state() const` — return home-button state
///
/// Both `espp::Gt911` and `espp::St7123Touch` satisfy this concept.
template <typename T>
concept TouchDriverConcept = requires(T &t, std::error_code &ec, uint8_t *n, uint16_t *x,
                                      uint16_t *y) {
  { t.update(ec) } -> std::convertible_to<bool>;
  {t.get_touch_point(n, x, y)};
  { t.get_home_button_state() } -> std::convertible_to<bool>;
};

/// @brief Abstract type-erased interface for a touch driver.
///
/// Used together with `TouchDriverAdapter<T>` to allow a BSP (such as the
/// M5Stack Tab5) or any other consumer to store a single
/// `std::shared_ptr<ITouchDriver>` regardless of which concrete driver
/// (Gt911, St7123Touch, …) is in use at runtime.
struct ITouchDriver {
  virtual ~ITouchDriver() = default;

  /// @brief Read new touch data from the hardware.
  /// @param ec Set on I2C error; cleared on success.
  /// @return True when new coordinate data is available.
  virtual bool update(std::error_code &ec) = 0;

  /// @brief Retrieve the primary touch point.
  /// @param num_touch_points Output: number of active touch points.
  /// @param x Output: X coordinate.
  /// @param y Output: Y coordinate.
  virtual void get_touch_point(uint8_t *num_touch_points, uint16_t *x, uint16_t *y) const = 0;

  /// @brief Return the home-button pressed state.
  virtual bool get_home_button_state() const = 0;
};

/// @brief Concept-constrained adapter that wraps any concrete touch driver
///        satisfying `TouchDriverConcept` behind the `ITouchDriver` interface.
///
/// Usage:
/// @code
///   auto driver = std::make_shared<espp::Gt911>(...);
///   std::shared_ptr<espp::ITouchDriver> touch =
///       std::make_shared<espp::TouchDriverAdapter<espp::Gt911>>(driver);
/// @endcode
template <TouchDriverConcept T> struct TouchDriverAdapter : ITouchDriver {
  /// Underlying concrete driver
  std::shared_ptr<T> driver;

  explicit TouchDriverAdapter(std::shared_ptr<T> d)
      : driver(std::move(d)) {}

  bool update(std::error_code &ec) override { return driver->update(ec); }

  void get_touch_point(uint8_t *num_touch_points, uint16_t *x, uint16_t *y) const override {
    driver->get_touch_point(num_touch_points, x, y);
  }

  bool get_home_button_state() const override { return driver->get_home_button_state(); }
};

/// @brief Convenience factory: wrap a shared_ptr to a concrete touch driver in
///        a `TouchDriverAdapter` and return it as `std::shared_ptr<ITouchDriver>`.
/// @tparam T Concrete driver type — must satisfy `TouchDriverConcept`.
template <TouchDriverConcept T>
std::shared_ptr<ITouchDriver> make_touch_driver(std::shared_ptr<T> driver) {
  return std::make_shared<TouchDriverAdapter<T>>(std::move(driver));
}
} // namespace espp
