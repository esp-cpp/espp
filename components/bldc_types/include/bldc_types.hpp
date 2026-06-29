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
/// \details VOLTAGE control requires no current sensing and is fully supported:
///      the torque command is converted to a q-axis voltage (using the phase
///      resistance and estimated back-EMF if those are provided). DC_CURRENT and
///      FOC_CURRENT close a current loop and therefore require a current sensor
///      implementing the CurrentSensorConcept (for example espp::CurrentSense
///      from the bldc_current_sense component) to be supplied to the BldcMotor.
/// \note DC_CURRENT and FOC_CURRENT are experimental: they depend on accurate,
///      PWM-synchronized current sampling and per-board calibration, so they
///      must be validated on hardware before use. If no current sensor is
///      provided, only VOLTAGE control is functional.
enum class TorqueControlType {
  VOLTAGE,    //!< Torque control using voltage (no current sense required)
  DC_CURRENT, //!< Torque control using DC current (one current magnitude; needs current sense)
  FOC_CURRENT //!< Torque control using DQ currents (needs current sense)
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

#include "format.hpp"

// for easy printing of enums using fmt
template <> struct fmt::formatter<espp::detail::MotionControlType> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::detail::MotionControlType t, FormatContext &ctx) const {
    switch (t) {
    case espp::detail::MotionControlType::TORQUE:
      return fmt::format_to(ctx.out(), "TORQUE");
    case espp::detail::MotionControlType::VELOCITY:
      return fmt::format_to(ctx.out(), "VELOCITY");
    case espp::detail::MotionControlType::ANGLE:
      return fmt::format_to(ctx.out(), "ANGLE");
    case espp::detail::MotionControlType::VELOCITY_OPENLOOP:
      return fmt::format_to(ctx.out(), "VELOCITY_OPENLOOP");
    case espp::detail::MotionControlType::ANGLE_OPENLOOP:
      return fmt::format_to(ctx.out(), "ANGLE_OPENLOOP");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

template <> struct fmt::formatter<espp::detail::TorqueControlType> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::detail::TorqueControlType t, FormatContext &ctx) const {
    switch (t) {
    case espp::detail::TorqueControlType::VOLTAGE:
      return fmt::format_to(ctx.out(), "VOLTAGE");
    case espp::detail::TorqueControlType::DC_CURRENT:
      return fmt::format_to(ctx.out(), "DC_CURRENT");
    case espp::detail::TorqueControlType::FOC_CURRENT:
      return fmt::format_to(ctx.out(), "FOC_CURRENT");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

template <> struct fmt::formatter<espp::detail::FocType> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext> auto format(espp::detail::FocType t, FormatContext &ctx) const {
    switch (t) {
    case espp::detail::FocType::SINE_PWM:
      return fmt::format_to(ctx.out(), "SINE_PWM");
    case espp::detail::FocType::SPACE_VECTOR_PWM:
      return fmt::format_to(ctx.out(), "SPACE_VECTOR_PWM");
    case espp::detail::FocType::TRAPEZOID_120:
      return fmt::format_to(ctx.out(), "TRAPEZOID_120");
    case espp::detail::FocType::TRAPEZOID_150:
      return fmt::format_to(ctx.out(), "TRAPEZOID_150");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};
