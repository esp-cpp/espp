/// @file bldc_types.hpp

#pragma once

/// @namespace espp
/// @brief espp namespace
namespace espp {
/// @namespace espp::detail
/// @brief detail namespace
namespace detail {

/// \brief The type of control loop the motor runs.
enum class MotionControlType {
  TORQUE,            //!< Torque-control (providing constant torque with current feedback).
  VELOCITY,          //!< Velocity closed-loop control, using speed feedback from the sensor.
  ANGLE,             //!< Angle closed-loop control, using angle feedback from the sensor.
  VELOCITY_OPENLOOP, //!< Velocity open-loop control, without feedback.
  ANGLE_OPENLOOP     //!< Angle open-loop control, without feedback.
};

/// \brief How the torque is controlled.
/// \note VOLTAGE is the only one supported right now, since the other two
///      require current sense.
enum class TorqueControlType {
  VOLTAGE,    //!< Torque control using voltage
  DC_CURRENT, //!< Torque control using DC current (one current magnitude)
  FOC_CURRENT //!< Torque control using DQ currents
};

/// \brief How the voltages / pwms are calculated based on the magnitude and
///       phase of the drive vector.
enum class FocType {
  SINE_PWM,         //!< Sinusoidal PWM modulation
  SPACE_VECTOR_PWM, //!< Space Vector modulation
  TRAPEZOID_120,    //!< 120 degree trapezoidal modulation
  TRAPEZOID_150     //!< 150 degree trapezoidal modulation
};
} // namespace detail
} // namespace espp
