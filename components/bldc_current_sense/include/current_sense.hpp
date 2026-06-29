#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "base_component.hpp"
#include "fast_math.hpp"

#include "bldc_driver.hpp"
#include "foc_utils.hpp"

namespace espp {

/// \brief Generic field-oriented-control (FOC) current sensor for a BLDC motor.
/// \details This class turns raw, per-phase current measurements (in amps) into
///          the quantities the FOC current loop needs: the d/q currents
///          (get_foc_currents()) and the signed DC current magnitude
///          (get_dc_current()). It implements the CurrentSensorConcept used by
///          espp::BldcMotor, so it can be supplied as the motor's current sense
///          to enable detail::TorqueControlType::DC_CURRENT and
///          detail::TorqueControlType::FOC_CURRENT control.
///
///          The class is intentionally decoupled from any specific ADC: you
///          provide a `read_phase_currents` callback that returns the most
///          recently sampled phase currents (already converted to amps). For
///          accurate low-side current sensing those samples must be taken while
///          the low-side FETs are conducting (the PWM center). Use
///          espp::BldcDriver::register_pwm_sample_callback() to trigger your ADC
///          read at that instant and have the `read_phase_currents` callback
///          return the latest result.
///
///          For boards that only measure two of the three phases, return NAN for
///          the unmeasured phase and it will be reconstructed assuming
///          Ia + Ib + Ic = 0.
///
/// \note This component is EXPERIMENTAL. It performs zero-current offset
///       calibration in driver_align(), but automatic phase-ordering / gain-sign
///       discovery is not yet implemented (provide phase_gain_signs in the
///       Config if your wiring inverts a channel). Current-mode torque control
///       depends on accurate, PWM-synchronized sampling and per-board tuning and
///       must be validated on hardware.
///
/// \tparam Driver The BLDC driver type (defaults to espp::BldcDriver). Only used
///         (optionally) during driver_align().
///
/// \section bldc_current_sense_ex1 Current Sense Example
/// \snippet bldc_current_sense_example.cpp bldc current sense example
template <typename Driver = espp::BldcDriver> class CurrentSense : public BaseComponent {
public:
  /// \brief Callback returning the most recently sampled phase currents in amps.
  /// \details The returned currents may include the analog bias/offset of the
  ///          sense circuit; the offset captured by driver_align() is subtracted
  ///          internally. Set any phase that is not physically measured to NAN
  ///          and it will be reconstructed from the others (Ia + Ib + Ic = 0).
  typedef std::function<PhaseCurrent()> read_phase_currents_fn;

  /// \brief Configuration for the current sense.
  struct Config {
    read_phase_currents_fn read_phase_currents{
        nullptr};                            ///< Callback returning raw phase currents (amps).
    std::shared_ptr<Driver> driver{nullptr}; ///< Optional driver, used by driver_align().
    size_t calibration_samples{500};         ///< Number of samples averaged for offset calibration.
    std::array<float, 3> phase_gain_signs{
        1.0f, 1.0f, 1.0f}; ///< Per-phase (a,b,c) gain sign to correct inverted sense channels.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log verbosity.
  };

  /// \brief Construct a new CurrentSense.
  /// \param config The configuration.
  explicit CurrentSense(const Config &config)
      : BaseComponent("CurrentSense", config.log_level)
      , read_phase_currents_(config.read_phase_currents)
      , driver_(config.driver)
      , calibration_samples_(config.calibration_samples)
      , gain_sign_(config.phase_gain_signs) {}

  /// \brief Align the current sense with the driver.
  /// \param voltage Alignment voltage (reserved for future active phase/gain
  ///        discovery; currently unused beyond logging).
  /// \return True if calibration succeeded.
  /// \details Captures the zero-current ADC offset for each measured phase by
  ///          averaging `calibration_samples` reads. The motor must be at rest
  ///          with no phase current while this runs (the BldcMotor calls this
  ///          after the phases have been driven to zero).
  /// \note Non-const because it mutates the captured calibration state, as
  ///       required by the CurrentSensorConcept.
  bool driver_align(float voltage) {
    if (!read_phase_currents_) {
      logger_.error("No read_phase_currents callback provided, cannot calibrate");
      return false;
    }
    logger_.info("Calibrating current sense offsets ({} samples, align voltage {:.2f})",
                 calibration_samples_, voltage);
    using namespace std::chrono_literals;
    double sum_a = 0, sum_b = 0, sum_c = 0;
    size_t n_a = 0, n_b = 0, n_c = 0;
    for (size_t i = 0; i < calibration_samples_; i++) {
      PhaseCurrent raw = read_phase_currents_();
      if (!std::isnan(raw.a)) {
        sum_a += raw.a;
        n_a++;
      }
      if (!std::isnan(raw.b)) {
        sum_b += raw.b;
        n_b++;
      }
      if (!std::isnan(raw.c)) {
        sum_c += raw.c;
        n_c++;
      }
      std::this_thread::sleep_for(1ms);
    }
    // if a phase was never measured, leave its offset at 0 (it is reconstructed)
    offset_.a = n_a ? (float)(sum_a / n_a) : 0.0f;
    offset_.b = n_b ? (float)(sum_b / n_b) : 0.0f;
    offset_.c = n_c ? (float)(sum_c / n_c) : 0.0f;
    if (n_a == 0 && n_b == 0 && n_c == 0) {
      logger_.error("No phase currents were measured during calibration");
      return false;
    }
    if ((n_a == 0) + (n_b == 0) + (n_c == 0) > 1) {
      logger_.error("Need at least two measured phases for FOC current sensing");
      return false;
    }
    logger_.info("Current sense offsets (A): a={:.4f} b={:.4f} c={:.4f}", offset_.a, offset_.b,
                 offset_.c);
    calibrated_ = true;
    return true;
  }

  /// \brief Get the calibrated phase currents (amps).
  /// \return Offset-removed, gain-sign-corrected phase currents, with the
  ///         unmeasured phase (if any) reconstructed from Ia + Ib + Ic = 0.
  PhaseCurrent get_phase_currents() const {
    PhaseCurrent raw = read_phase_currents_ ? read_phase_currents_() : PhaseCurrent{NAN, NAN, NAN};
    float a = (raw.a - offset_.a) * gain_sign_[0];
    float b = (raw.b - offset_.b) * gain_sign_[1];
    float c = (raw.c - offset_.c) * gain_sign_[2];
    // reconstruct a single unmeasured (NAN) phase assuming Ia + Ib + Ic = 0
    const bool na = std::isnan(a), nb = std::isnan(b), nc = std::isnan(c);
    const int num_nan = (int)na + (int)nb + (int)nc;
    if (num_nan == 1) {
      if (na)
        a = -(b + c);
      else if (nb)
        b = -(a + c);
      else
        c = -(a + b);
    }
    return {a, b, c};
  }

  /// \brief Get the d/q currents for the given electrical angle.
  /// \param electrical_angle The motor electrical angle (radians).
  /// \return The d (direct) and q (quadrature) currents in amps.
  DqCurrent get_foc_currents(float electrical_angle) const {
    float i_alpha, i_beta;
    clarke(get_phase_currents(), i_alpha, i_beta);
    // Park transform
    const float ct = fast_cos(electrical_angle);
    const float st = fast_sin(electrical_angle);
    DqCurrent dq;
    dq.d = i_alpha * ct + i_beta * st;
    dq.q = i_beta * ct - i_alpha * st;
    return dq;
  }

  /// \brief Get the signed DC current magnitude.
  /// \param electrical_angle The motor electrical angle (radians), used to give
  ///        the magnitude a sign relative to the drive (q) direction.
  /// \return The signed current magnitude in amps.
  float get_dc_current(float electrical_angle) const {
    float i_alpha, i_beta;
    clarke(get_phase_currents(), i_alpha, i_beta);
    const float magnitude = std::sqrt(i_alpha * i_alpha + i_beta * i_beta);
    // sign of the q-axis (drive direction) current
    const float i_q = i_beta * fast_cos(electrical_angle) - i_alpha * fast_sin(electrical_angle);
    return (i_q >= 0.0f ? 1.0f : -1.0f) * magnitude;
  }

  /// \brief Whether the current sense has been calibrated (driver_align ran).
  /// \return True if calibrated.
  bool is_calibrated() const { return calibrated_; }

protected:
  /// \brief Amplitude-invariant Clarke transform (abc -> alpha/beta).
  /// \details Removes the common-mode (zero-sequence) component before
  ///          projecting, so it is robust whether all three phases are measured
  ///          or the third was reconstructed.
  void clarke(const PhaseCurrent &i, float &i_alpha, float &i_beta) const {
    const float mid = (1.0f / 3.0f) * (i.a + i.b + i.c);
    const float a = i.a - mid;
    const float b = i.b - mid;
    i_alpha = a;
    i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
  }

  read_phase_currents_fn read_phase_currents_{nullptr};
  std::shared_ptr<Driver> driver_{nullptr};
  size_t calibration_samples_{500};
  PhaseCurrent offset_{0.0f, 0.0f, 0.0f};
  std::array<float, 3> gain_sign_{1.0f, 1.0f, 1.0f};
  bool calibrated_{false};
};

} // namespace espp
