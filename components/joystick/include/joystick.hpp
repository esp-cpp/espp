#pragma once

#include <functional>

#include "base_component.hpp"
#include "range_mapper.hpp"
#include "vector2d.hpp"

namespace espp {
/**
 *  @brief 2-axis Joystick with axis mapping / calibration.
 *
 * \section joystick_ex1 ADC Joystick Example
 * \snippet joystick_example.cpp adc joystick example
 */
class Joystick : public BaseComponent {
public:
  /**
   * @brief Type of the joystick.
   * @note When using a Type::CIRCULAR joystick, it's recommended to set the
   *       individual x/y calibration deadzones to be 0 and to only use the
   *       deadzone_radius field to set the deadzone around the center.
   */
  enum class Type {
    RECTANGULAR, ///< The default type of joystick. Uses the rangemappers for
                 ///  each axis (to convert raw values from input range to be
                 ///  [-1,1]) independently which results in x/y deadzones and
                 ///  output that are rectangular.
    CIRCULAR,    ///< The joystick is configured to have a circular output. This
                 ///  means that the x/y < deadzones are circular around the input
                 ///  and the output is clamped to be on or within the unit circle.
  };

  /**
   * @brief function for gettin x/y values for the joystick.
   * @param x Pointer to the x value. Function should fill this variable
   *          with the latest reading.
   * @param y Pointer to the y value. Function should fill this variable
   *          with the latest reading.
   * @return True if the values were able to be retrieved, false otherwise.
   */
  typedef std::function<bool(float *x, float *y)> get_values_fn;

  /**
   *  @brief Configuration structure for the joystick.
   */
  struct Config {
    FloatRangeMapper::Config x_calibration; /**< Configuration for the x axis. */
    FloatRangeMapper::Config y_calibration; /**< Configuration for the y axis. */
    Type type{Type::RECTANGULAR};           /**< The type of the joystick. See Type enum
                                                           for more information. */
    float center_deadzone_radius{
        0}; /**< The radius of the unit circle's deadzone [0, 1.0f] around the center, only used
        when the joystick is configured as Type::CIRCULAR. */
    get_values_fn
        get_values; /**< Function to retrieve the latest unmapped joystick values (range [-1,1]). */
    Logger::Verbosity log_level{
        Logger::Verbosity::WARN}; /**< Verbosity for the Joystick logger_. */
  };

  /**
   *  @brief Initalize the joystick using the provided configuration.
   *  @param config Config structure with initialization information.
   */
  explicit Joystick(const Config &config)
      : BaseComponent("Joystick", config.log_level)
      , x_mapper_(config.x_calibration)
      , y_mapper_(config.y_calibration)
      , type_(config.type)
      , center_deadzone_radius_(config.center_deadzone_radius)
      , get_values_(config.get_values) {}

  /**
   *  @brief Set the type of the joystick.
   *  @param type The Type of the joystick.
   *  @param radius Optional radius parameter used when \p type is
   *         Type::CIRCULAR. When the magnitude of the joystick's mapped
   *         position vector is less than this value, the vector is set to
   *         (0,0).
   *  @note If the Joystick is Type::CIRCULAR, the actual calibrations that are
   *        saved into the joystick will have 0 deadzone around the center
   *        value, so that only the center deadzone radius is applied.
   *  @sa set_center_deadzone_radius
   *  @sa set_calibration
   */
  void set_type(Type type, float radius = 0) {
    type_ = type;
    if (type_ == Type::CIRCULAR) {
      x_mapper_.set_deadband(0);
      y_mapper_.set_deadband(0);
    }
    center_deadzone_radius_ = radius;
  }

  /**
   * @brief Sets the center deadzone radius.
   * @note Radius is only applied when \p deadzone is Deadzone::CIRCULAR.
   * @param radius Optional radius parameter used when \p deadzone is
   *        Deadzone::CIRCULAR. When the magnitude of the joystick's mapped
   *        position vector is less than this value, the vector is set to
   *        (0,0).
   */
  void set_center_deadzone_radius(float radius) { center_deadzone_radius_ = radius; }

  /**
   * @brief Update the x and y axis mapping.
   * @param x_calibration New x-axis range mapping configuration to use.
   * @param y_calibration New y-axis range mapping configuration to use.
   * @param center_deadzone_radius The radius of the unit circle's deadzone [0,
   *        1.0f] around the center, only used when the joystick is configured
   *        as Type::CIRCULAR.
   * @note If the Joystick is Type::CIRCULAR, the actual calibrations that are
   *       saved into the joystick will have 0 deadzone around the center value,
   *       so that only the center deadzone radius is applied.
   * @sa set_center_deadzone_radius
   */
  void set_calibration(const FloatRangeMapper::Config &x_calibration,
                       const FloatRangeMapper::Config &y_calibration,
                       float center_deadzone_radius = 0) {
    x_mapper_.configure(x_calibration);
    y_mapper_.configure(y_calibration);
    if (type_ == Type::CIRCULAR) {
      x_mapper_.set_deadband(0);
      y_mapper_.set_deadband(0);
    }
    center_deadzone_radius_ = center_deadzone_radius;
  }

  /**
   * @brief Read the raw values and use the calibration data to update the
   *        position.
   */
  void update() {
    if (!get_values_) {
      logger_.error("No function provided with which to get values!");
      return;
    }
    float _x, _y;
    logger_.info("Getting x,y values");
    bool success = get_values_(&_x, &_y);
    if (!success) {
      logger_.error("Could not get values!");
      return;
    }
    logger_.debug("Got x,y values: ({}, {})", _x, _y);
    raw_.x(_x);
    raw_.y(_y);
    position_.x(x_mapper_.map(_x));
    position_.y(y_mapper_.map(_y));
    // if we're configured to be a Type::CIRCULAR joystick, use the center
    // deadzone radius and clamp the output to be within the unit circle
    if (type_ == Type::CIRCULAR) {
      auto magnitude = position_.magnitude();
      if (magnitude < center_deadzone_radius_) {
        // if it's within the deadzone radius, then set both axes to 0.
        position_.x(0);
        position_.y(0);
      } else if (magnitude > 1.0f) {
        // if it's outside the unit circle, then normalize the vector so that
        // it's on the unit circle
        position_ = position_.normalized();
      } else {
        // otherwise we should scale the vector so that it's 0 on the edge of
        // the deadzone and 1 on the edge of the unit circle
        const float magnitude_range = 1.0f - center_deadzone_radius_;
        const float scale = (magnitude - center_deadzone_radius_) / magnitude_range;
        position_ *= scale;
      }
    }
  }

  /**
   * @brief Get the most recently updated x axis calibrated position.
   * @return The most recent x-axis position (from when update() was last
   *         called).
   */
  float x() const { return position_.x(); }

  /**
   * @brief Get the most recently updated y axis calibrated position.
   * @return The most recent y-axis position (from when update() was last
   *         called).
   */
  float y() const { return position_.y(); }

  /**
   * @brief Get the most recently updated calibrated position.
   * @return The most recent position (from when update() was last called).
   */
  Vector2f position() const { return position_; }

  /**
   * @brief Get the most recently updated raw / uncalibrated readings. This
   *        function is useful for externally performing a calibration routine
   *        and creating updated calibration / mapper configuration
   *        structures.
   * @return The most recent raw measurements (from when update() was last
   *         called).
   */
  Vector2f raw() const { return raw_; }

  friend struct fmt::formatter<Joystick>;

protected:
  Vector2f raw_{};
  Vector2f position_{};
  FloatRangeMapper x_mapper_{};
  FloatRangeMapper y_mapper_{};
  Type type_{Type::RECTANGULAR};
  float center_deadzone_radius_{0};
  get_values_fn get_values_{nullptr};
};
} // namespace espp

// for allowing easy serialization/printing of the
// Trigger class
template <> struct fmt::formatter<espp::Joystick> {
  // Presentation format: 'v' - value, 'r' - raw, 'b' - both.
  char presentation = 'v';

  // Parses format specifications of the form ['v' | 'r' | 'b'].
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) {
    // Parse the presentation format and store it in the formatter:
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'v' || *it == 'r' || *it == 'b'))
      presentation = *it++;

    // TODO: Check if reached the end of the range:
    // if (it != end && *it != '}') throw format_error("invalid format");

    // Return an iterator past the end of the parsed range:
    return it;
  }

  template <typename FormatContext> auto format(espp::Joystick const &j, FormatContext &ctx) const {
    switch (presentation) {
    case 'v':
      return fmt::format_to(ctx.out(), "{}", j.position_);
    case 'r':
      return fmt::format_to(ctx.out(), "{}", j.raw_);
    case 'b':
      return fmt::format_to(ctx.out(), "({} -> {})", j.raw_, j.position_);
    default:
      // shouldn't get here!
      return fmt::format_to(ctx.out(), "{}", j.position_);
    }
  }
};
