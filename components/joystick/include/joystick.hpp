#pragma once

#include <functional>

#include "logger.hpp"
#include "range_mapper.hpp"
#include "vector2d.hpp"

namespace espp {
  /**
   *  @brief 2-axis Joystick with axis mapping / calibration.
   *
   * \section joystick_ex1 ADC Joystick Example
   * \snippet joystick_example.cpp adc joystick example
   */
  class Joystick {
  public:
    /**
     * @brief function for gettin x/y values for the joystick. Values should be
     *        in the range [-1,1].
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
      get_values_fn get_values; /**< Function to retrieve the latest unmapped joystick values (range [-1,1]). */
      Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Verbosity for the Joystick logger_. */
    };

    /**
     *  @brief Initalize the joystick using the provided configuration.
     *  @param config Config structure with initialization information.
     */
    Joystick(const Config& config)
      : x_mapper_(config.x_calibration),
        y_mapper_(config.y_calibration),
        get_values_(config.get_values),
        logger_({.tag = "Joystick", .level = config.log_level}) {
    }

    /**
     * @brief Update the x and y axis mapping.
     * @param x_calibration New x-axis range mapping configuration to use.
     * @param y_calibration New y-axis range mapping configuration to use.
     */
    void set_calibration(const FloatRangeMapper::Config& x_calibration, const FloatRangeMapper::Config& y_calibration) {
      x_mapper_.configure(x_calibration);
      y_mapper_.configure(y_calibration);
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
      float x,y;
      logger_.info("Getting x,y values");
      bool success = get_values_(&x, &y);
      if (!success) {
        logger_.error("Could not get values!");
        return;
      }
      logger_.debug("Got x,y values: ({}, {})", x, y);
      raw_.x(x);
      raw_.y(y);
      position_.x(x_mapper_.map(x));
      position_.y(y_mapper_.map(y));
    }

    /**
     * @brief Get the most recently updated x axis calibrated position.
     * @return The most recent x-axis position (from when update() was last
     *         called).
     */
    float x() const {
      return position_.x();
    }

    /**
     * @brief Get the most recently updated y axis calibrated position.
     * @return The most recent y-axis position (from when update() was last
     *         called).
     */
    float y() const {
      return position_.y();
    }

    /**
     * @brief Get the most recently updated calibrated position.
     * @return The most recent position (from when update() was last called).
     */
    Vector2f position() const {
      return position_;
    }

    /**
     * @brief Get the most recently updated raw / uncalibrated readings. This
     *        function is useful for externally performing a calibration routine
     *        and creating updated calibration / mapper configuration
     *        structures.
     * @return The most recent raw measurements (from when update() was last
     *         called).
     */
    Vector2f raw() const {
      return raw_;
    }

  protected:
    Vector2f raw_;
    Vector2f position_;
    FloatRangeMapper x_mapper_;
    FloatRangeMapper y_mapper_;
    get_values_fn get_values_;
    Logger logger_;
  };
}
