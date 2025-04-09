#pragma once

#include <functional>

#include "base_component.hpp"
#include "range_mapper.hpp"
#include "vector2d.hpp"

namespace espp {
/**
 *  @brief 2-axis Joystick with axis mapping / calibration.
 *
 * \section joystick_ex1 Basic Circular and Rectangular Joystick Example
 * \snippet joystick_example.cpp circular joystick example
 * \section joystick_ex2 ADC Joystick Example
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
                 ///  means that the x/y < deadzones are circular around the
                 ///  input and range and the output is clamped to be on or
                 ///  within the unit circle.
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
    espp::FloatRangeMapper::Config x_calibration; /**< Configuration for the x axis. */
    espp::FloatRangeMapper::Config y_calibration; /**< Configuration for the y axis. */
    espp::Joystick::Type type{espp::Joystick::Type::RECTANGULAR}; /**< The type of the joystick. See
                                                 Type enum for more information. */
    float center_deadzone_radius{
        0}; /**< The radius of the unit circle's deadzone [0, 1.0f] around the center, only used
        when the joystick is configured as Type::CIRCULAR. */
    float range_deadzone{0}; /**< The deadzone around the edge of the unit circle, only used when
                          the joystick is configured as Type::CIRCULAR. This scales the output so
                          that the output appears to have magnitude 1 (meaning it appears to be on
                          the edge of the unit circle) when the joystick value magnitude is within
                          the range [1-range_deadzone, 1]. */
    espp::Joystick::get_values_fn get_values{nullptr}; /**< Function to retrieve the latest
                                          unmapped joystick values. Required if
                                          you want to use update(), unused if
                                          you call update(float raw_x, float
                                          raw_y). */
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; /**< Verbosity for the Joystick logger_. */
  };

  /**
   *  @brief Initalize the joystick using the provided configuration.
   *  @param config Config structure with initialization information.
   */
  explicit Joystick(const espp::Joystick::Config &config);

  /**
   *  @brief Set the type of the joystick.
   *  @param type The Type of the joystick.
   *  @param radius Optional radius parameter used when \p type is
   *         Type::CIRCULAR. When the magnitude of the joystick's mapped
   *         position vector is less than this value, the vector is set to
   *         (0,0).
   *  @param range_deadzone Optional deadzone around the edge of the unit circle
   *         when \p type is Type::CIRCULAR. This scales the output so that the
   *         output appears to have magnitude 1 (meaning it appears to be on the
   *         edge of the unit circle) if the magnitude of the mapped position
   *         vector is greater than 1-range_deadzone. Example: if the range
   *         deadzone is 0.1, then the output will be scaled so that the
   *         magnitude of the output is 1 if the magnitude of the mapped
   *         position vector is greater than 0.9.
   *  @note If the Joystick is Type::CIRCULAR, the actual calibrations that are
   *        saved into the joystick will have 0 deadzone around the center value
   *        and range values, so that center and range deadzones are actually
   *        applied on the vector value instead of on the individual axes
   *        independently.
   *  @sa set_center_deadzone_radius
   *  @sa set_range_deadzone
   *  @sa set_calibration
   */
  void set_type(espp::Joystick::Type type, float radius = 0, float range_deadzone = 0);

  /**
   * @brief Get the type of the joystick.
   * @return The Type of the joystick.
   */
  espp::Joystick::Type type() const;

  /**
   * @brief Sets the center deadzone radius.
   * @note Radius is only applied when \p deadzone is Deadzone::CIRCULAR.
   * @param radius Optional radius parameter used when \p deadzone is
   *        Deadzone::CIRCULAR. When the magnitude of the joystick's mapped
   *        position vector is less than this value, the vector is set to
   *        (0,0).
   */
  void set_center_deadzone_radius(float radius);

  /**
   * @brief Get the center deadzone radius.
   * @return The center deadzone radius.
   */
  float center_deadzone_radius() const;

  /**
   * @brief Sets the range deadzone.
   * @note Range deadzone is only applied when \p deadzone is Deadzone::CIRCULAR.
   * @param range_deadzone Optional deadzone around the edge of the unit circle
   *        when \p deadzone is Deadzone::CIRCULAR. This scales the output so
   *        that the output appears to have magnitude 1 (meaning it appears to
   *        be on the edge of the unit circle) if the magnitude of the mapped
   *        position vector is greater than 1-range_deadzone. Example: if the
   *        range deadzone is 0.1, then the output will be scaled so that the
   *        magnitude of the output is 1 if the magnitude of the mapped position
   *        vector is greater than 0.9.
   */
  void set_range_deadzone(float range_deadzone);

  /**
   * @brief Get the range deadzone.
   * @return The range deadzone.
   */
  float range_deadzone() const;

  /**
   * @brief Update the x and y axis mapping.
   * @param x_calibration New x-axis range mapping configuration to use.
   * @param y_calibration New y-axis range mapping configuration to use.
   * @param center_deadzone_radius The radius of the unit circle's deadzone [0,
   *        1.0f] around the center, only used when the joystick is configured
   *        as Type::CIRCULAR.
   *  @param range_deadzone Optional deadzone around the edge of the unit circle
   *         when \p type is Type::CIRCULAR. This scales the output so that the
   *         output appears to have magnitude 1 (meaning it appears to be on the
   *         edge of the unit circle) if the magnitude of the mapped position
   *         vector is greater than 1-range_deadzone. Example: if the range
   *         deadzone is 0.1, then the output will be scaled so that the
   *         magnitude of the output is 1 if the magnitude of the mapped
   *         position vector is greater than 0.9.
   * @note If the Joystick is Type::CIRCULAR, the actual calibrations that are
   *       saved into the joystick will have 0 deadzone around the center and range values,
   *       so that center and range deadzones are actually applied on the vector value.
   * @sa set_center_deadzone_radius
   * @sa set_range_deadzone
   */
  void set_calibration(const espp::FloatRangeMapper::Config &x_calibration,
                       const espp::FloatRangeMapper::Config &y_calibration,
                       float center_deadzone_radius = 0, float range_deadzone = 0);

  /**
   * @brief Read the raw values and use the calibration data to update the
   *        position.
   * @note Requires that the get_values_ function is set.
   */
  void update();

  /**
   * @brief Update the joystick's position using the provided raw x and y
   *        values.
   * @param raw_x The raw x-axis value.
   * @param raw_y The raw y-axis value.
   * @note This function is useful when you have the raw values and don't want
   *       to use the get_values_ function.
   */
  void update(float raw_x, float raw_y);

  /**
   * @brief Get the most recently updated x axis calibrated position.
   * @return The most recent x-axis position (from when update() was last
   *         called).
   */
  float x() const;

  /**
   * @brief Get the most recently updated y axis calibrated position.
   * @return The most recent y-axis position (from when update() was last
   *         called).
   */
  float y() const;

  /**
   * @brief Get the most recently updated calibrated position.
   * @return The most recent position (from when update() was last called).
   * @note The returned reference is valid as long as the Joystick object is alive.
   */
  const espp::Vector2f &position() const;

  /**
   * @brief Get the most recently updated raw / uncalibrated readings. This
   *        function is useful for externally performing a calibration routine
   *        and creating updated calibration / mapper configuration
   *        structures.
   * @return The most recent raw measurements (from when update() was last
   *         called).
   * @note The returned reference is valid as long as the Joystick object is alive.
   */
  const espp::Vector2f &raw() const;

  friend struct fmt::formatter<Joystick>;

protected:
  void recalculate(float raw_x, float raw_y);

  Vector2f raw_{};
  Vector2f position_{};
  FloatRangeMapper x_mapper_{};
  FloatRangeMapper y_mapper_{};
  Type type_{Type::RECTANGULAR};
  float center_deadzone_radius_{0};
  float range_deadzone_{0};
  get_values_fn get_values_{nullptr};
};
} // namespace espp

#include "joystick_formatters.hpp"
