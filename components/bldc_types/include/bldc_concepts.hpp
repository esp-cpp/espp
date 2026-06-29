#pragma once

#include <system_error>

#include "bldc_types.hpp"
#include "foc_utils.hpp"

namespace espp {

/**
 * @brief Concept defining the required interfaces for the Driver for a BLDC Motor.
 */
template <class FOO>
concept DriverConcept = requires {
  static_cast<void (FOO::*)(void)>(&FOO::enable);
  static_cast<void (FOO::*)(void)>(&FOO::disable);
  static_cast<void (FOO::*)(float, float, float)>(&FOO::set_voltage);
  static_cast<void (FOO::*)(int, int, int)>(&FOO::set_phase_state);
  static_cast<float (FOO::*)(void) const>(&FOO::get_voltage_limit);
};

/**
 * @brief Concept defining the required interfaces for a Sensor on a BLDC Motor.
 */
template <class FOO>
concept SensorConcept = requires {
  static_cast<void (FOO::*)(std::error_code &)>(&FOO::update);
  static_cast<bool (FOO::*)(void) const>(&FOO::needs_zero_search);
  static_cast<float (FOO::*)(void) const>(&FOO::get_radians);
  static_cast<float (FOO::*)(void) const>(&FOO::get_rpm);
  static_cast<float (FOO::*)(void) const>(&FOO::get_mechanical_radians);
};

/**
 * @brief Concept defining the required interfaces for a Current Sensor on a BLDC Motor.
 */
// NOTE: driver_align() is intentionally non-const: a real current sensor must
// capture zero-current offsets and phase/gain calibration while aligning, which
// mutates the sensor's state. The reading methods remain const.
template <class FOO>
concept CurrentSensorConcept = requires {
  static_cast<float (FOO::*)(float) const>(&FOO::get_dc_current);
  static_cast<DqCurrent (FOO::*)(float) const>(&FOO::get_foc_currents);
  static_cast<bool (FOO::*)(float)>(&FOO::driver_align);
};

/**
 * @brief Concept defining the required interfaces for a Motor used for haptics.
 */
template <class FOO>
concept MotorConcept = requires {
  static_cast<void (FOO::*)(void)>(&FOO::enable);  ///< Enable the motor
  static_cast<void (FOO::*)(void)>(&FOO::disable); ///< Disable the motor
  static_cast<void (FOO::*)(float)>(
      &FOO::move); ///< Move the motor to a new target (position, velocity, or torque depending on
                   ///< the motor control type)
  static_cast<void (FOO::*)(detail::MotionControlType)>(
      &FOO::set_motion_control_type);                             ///< Set the motion control type
  static_cast<void (FOO::*)(void)>(&FOO::loop_foc);               ///< Run the FOC loop
  static_cast<float (FOO::*)(void) const>(&FOO::get_shaft_angle); ///< Get the shaft angle
  static_cast<float (FOO::*)(void) const>(&FOO::get_shaft_velocity); ///< Get the shaft velocity
  static_cast<float (FOO::*)(void)>(&FOO::get_electrical_angle);     ///< Get the electrical angle
};

// Provide a dummy current sense type here (as default) if you don't want to
// use (or your hardware doesn't support) an actual current sensor for FOC.
struct DummyCurrentSense {
  float get_dc_current(float) const { return 0.0f; }
  DqCurrent get_foc_currents(float) const { return {0.0f, 0.0f}; }
  bool driver_align(float v) { return true; }
};

} // namespace espp
