#pragma once

#include <atomic>

#include "fast_math.hpp"
#include "logger.hpp"
#include "pid.hpp"
#include "task.hpp"

#include "bldc_types.hpp"
#include "foc_utils.hpp"
#include "sensor_direction.hpp"

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
  static_cast<bool (FOO::*)(void) const>(&FOO::needs_zero_search);
  static_cast<float (FOO::*)(void) const>(&FOO::get_radians);
  static_cast<float (FOO::*)(void) const>(&FOO::get_rpm);
  static_cast<float (FOO::*)(void) const>(&FOO::get_mechanical_radians);
};

/**
 * @brief Concept defining the required interfaces for a Current Sensor on a BLDC Motor.
 */
template <class FOO>
concept CurrentSensorConcept = requires {
  static_cast<float (FOO::*)(float) const>(&FOO::get_dc_current);
  static_cast<DqCurrent (FOO::*)(float) const>(&FOO::get_foc_currents);
  static_cast<bool (FOO::*)(float) const>(&FOO::driver_align);
};

// Provide a dummy current sense type here (as default) if you don't want to
// use (or your hardware doesn't support) an actual current sensor for FOC.
struct DummyCurrentSense {
  float get_dc_current(float) const { return 0.0f; }
  DqCurrent get_foc_currents(float) const { return {0.0f, 0.0f}; }
  bool driver_align(float v) const { return true; }
};

/**
 * @brief Motor control class for a Brushless DC (BLDC) motor, implementing
 *        the field-oriented control (FOC) algorithm. Must be provided a
 *        driver object / type, and optionally a position/velocity sensor
 *        object/type and optionally a current sensor object / type.
 * @note This is a port (with some modifications) of the excellent work by
 *       SimpleFOC - https://simplefoc.com
 * @section bldc_motor_usage Example Usage
 * @snippet bldc_motor_example.cpp bldc_motor example
 */
template <DriverConcept D, SensorConcept S, CurrentSensorConcept CS = DummyCurrentSense>
class BldcMotor {
public:
  /**
   * @brief Filter the raw input sample and return it.
   * @param raw Most recent raw sample measured.
   * @return Filtered output from the input.
   */
  typedef std::function<float(float raw)> filter_fn;

  /**
   * @brief BLDC Motor / FOC configuration structure
   */
  struct Config {
    size_t num_pole_pairs;         /**< Number of pole pairs in the motor. */
    float phase_resistance;        /**< Motor phase resistance (ohms). */
    float kv_rating;               /**< Motor KV rating (1/K_bemf) - rpm/V */
    float phase_inductance{0};     /**< Motor phase inductance (Henries). */
    float current_limit{1.0f};     /**< Current limit (Amps) for the controller. */
    float velocity_limit{1000.0f}; /**< Velocity limit (RPM) for the controller. */
    float zero_electric_offset{0.0f};
    detail::SensorDirection sensor_direction{detail::SensorDirection::CLOCKWISE};
    detail::FocType foc_type{detail::FocType::SPACE_VECTOR_PWM}; /**< How the voltage for the phases
                                                                    should be calculated. */
    detail::TorqueControlType torque_controller{
        detail::TorqueControlType::VOLTAGE}; /**< Torque controller type. */
    std::shared_ptr<D> driver;               /**< Driver for low-level setting of phase PWMs. */
    std::shared_ptr<S> sensor;               /**< Sensor for measuring position / speed. */
    std::shared_ptr<CS> current_sense{
        nullptr}; /**< Sensor for measuring current through the motor. */
    Pid::Config current_pid_config{
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .integrator_min = 0,
        .integrator_max = 0,
        .output_min = 0,
        .output_max = 0}; /**< PID configuration for current (amps) pid controller. */
    Pid::Config velocity_pid_config{.kp = 0,
                                    .ki = 0,
                                    .kd = 0,
                                    .integrator_min = 0,
                                    .integrator_max = 0,
                                    .output_min = 0,
                                    .output_max =
                                        0}; /**< PID configuration for velocity pid controller. */
    Pid::Config angle_pid_config{.kp = 0,
                                 .ki = 0,
                                 .kd = 0,
                                 .integrator_min = 0,
                                 .integrator_max = 0,
                                 .output_min = 0,
                                 .output_max =
                                     0}; /**< PID configuration for angle pid controller. */
    filter_fn q_current_filter{nullptr}; /**< Optional filter added to the sensed q current. */
    filter_fn d_current_filter{nullptr}; /**< Optional filter added to the sensed d current. */
    filter_fn velocity_filter{nullptr};  /**< Optional filter added to the sensed velocity. */
    filter_fn angle_filter{nullptr};     /**< Optional filter added to the sensed angle. */
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Log verbosity for the Motor.  */
  };

  /**
   * @brief Create and initialize the BLDC motor, running through any
   *        necessary sensor calibration.
   */
  explicit BldcMotor(const Config &config)
      : num_pole_pairs_(config.num_pole_pairs), phase_resistance_(config.phase_resistance),
        phase_inductance_(config.phase_inductance), kv_rating_(config.kv_rating * _SQRT2),
        current_limit_(config.current_limit), velocity_limit_(config.velocity_limit),
        sensor_direction_(config.sensor_direction), foc_type_(config.foc_type),
        torque_control_type_(config.torque_controller), driver_(config.driver),
        sensor_(config.sensor), current_sense_(config.current_sense),
        pid_current_q_(config.current_pid_config), pid_current_d_(config.current_pid_config),
        pid_velocity_(config.current_pid_config), pid_angle_(config.current_pid_config),
        q_current_filter_(config.q_current_filter), d_current_filter_(config.d_current_filter),
        velocity_filter_(config.velocity_filter), angle_filter_(config.angle_filter),
        logger_({.tag = "BldcMotor", .level = config.log_level}) {
    // initialize the voltage limit
    voltage_limit_ = driver_->get_voltage_limit();
    voltage_sensor_align_ = voltage_limit_ / 4.0f;

    // update the pid limits
    auto current_pid_config = config.current_pid_config;
    current_pid_config.output_min = -voltage_limit_;
    current_pid_config.output_max = voltage_limit_;
    pid_current_q_.change_gains(current_pid_config);
    pid_current_d_.change_gains(current_pid_config);

    auto velocity_pid_config = config.velocity_pid_config;
    if (phase_resistance_ > 0 || torque_control_type_ != detail::TorqueControlType::VOLTAGE) {
      velocity_pid_config.output_min = -current_limit_;
      velocity_pid_config.output_max = current_limit_;
    } else {
      velocity_pid_config.output_min = -voltage_limit_;
      velocity_pid_config.output_max = voltage_limit_;
    }
    pid_velocity_.change_gains(velocity_pid_config);

    auto angle_pid_config = config.angle_pid_config;
    angle_pid_config.output_min = -velocity_limit_;
    angle_pid_config.output_max = velocity_limit_;
    pid_angle_.change_gains(angle_pid_config);

    // finish the rest of init
    init();
    // enable the motor for foc initialization
    enable();
    // then initialize the foc (calibration etc.)
    init_foc(config.zero_electric_offset, config.sensor_direction);
    // disable the motor to put it back into a safe state
    disable();
  }

  /**
   * @brief Destroy the motor, making sure to disable it first to ensure power
   *        is cutoff.
   */
  ~BldcMotor() { disable(); }

  /**
   * @brief Check if the motor is enabled.
   * @return True if the motor is enabled, false otherwise.
   */
  bool is_enabled() const { return enabled_; }

  /**
   * @brief Enable the controller and driver output.
   */
  void enable() {
    driver_->enable();
    enabled_ = true;
  }

  /**
   * @brief Disable the controller and driver output.
   */
  void disable() {
    enabled_ = false;
    driver_->disable();
  }

  /**
   * @brief Update the motoion control scheme the motor control loop uses.
   * @param motion_control_type New motion control to use.
   */
  void set_motion_control_type(detail::MotionControlType motion_control_type) {
    motion_control_type_ = motion_control_type;
  }

  /**
   * @brief Method using FOC to set Uq to the motor at the optimal angle. Heart
   *        of the FOC algorithm
   * @param uq Current voltage in q axis to set to the motor
   * @param ud Current voltage in d axis to set to the motor
   * @param el_angle current electrical angle of the motor
   */
  void set_phase_voltage(float uq, float ud, float el_angle) {
    if (!enabled_) {
      return;
    }

    float center;
    int sector;
    float _ca, _sa;

    switch (foc_type_) {
    case detail::FocType::TRAPEZOID_120:
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
      static int trap_120_map[6][3] = {
          {_HIGH_IMPEDANCE, 1, -1},
          {-1, 1, _HIGH_IMPEDANCE},
          {-1, _HIGH_IMPEDANCE, 1},
          {_HIGH_IMPEDANCE, -1, 1},
          {1, -1, _HIGH_IMPEDANCE},
          {1, _HIGH_IMPEDANCE,
           -1} // each is 60 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
      };
      // static int trap_120_state = 0;
      sector =
          6 * (normalize_angle(el_angle + _PI_6) / _2PI); // adding PI/6 to align with other modes
      // centering the voltages around either
      // modulation_centered_ == true > driver.volage_limit/2
      // modulation_centered_ == false > or Adaptable centering, all phases drawn to 0 when uq=0
      center = modulation_centered_ ? (driver_->get_voltage_limit()) / 2 : uq;

      if (trap_120_map[sector][0] == _HIGH_IMPEDANCE) {
        Ua = center;
        Ub = trap_120_map[sector][1] * uq + center;
        Uc = trap_120_map[sector][2] * uq + center;
        driver_->set_phase_state(_HIGH_IMPEDANCE, _ACTIVE, _ACTIVE); // disable phase if possible
      } else if (trap_120_map[sector][1] == _HIGH_IMPEDANCE) {
        Ua = trap_120_map[sector][0] * uq + center;
        Ub = center;
        Uc = trap_120_map[sector][2] * uq + center;
        driver_->set_phase_state(_ACTIVE, _HIGH_IMPEDANCE, _ACTIVE); // disable phase if possible
      } else {
        Ua = trap_120_map[sector][0] * uq + center;
        Ub = trap_120_map[sector][1] * uq + center;
        Uc = center;
        driver_->set_phase_state(_ACTIVE, _ACTIVE, _HIGH_IMPEDANCE); // disable phase if possible
      }
      break;

    case detail::FocType::TRAPEZOID_150:
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
      static int trap_150_map[12][3] = {
          {_HIGH_IMPEDANCE, 1, -1},
          {-1, 1, -1},
          {-1, 1, _HIGH_IMPEDANCE},
          {-1, 1, 1},
          {-1, _HIGH_IMPEDANCE, 1},
          {-1, -1, 1},
          {_HIGH_IMPEDANCE, -1, 1},
          {1, -1, 1},
          {1, -1, _HIGH_IMPEDANCE},
          {1, -1, -1},
          {1, _HIGH_IMPEDANCE, -1},
          {1, 1,
           -1} // each is 30 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
      };
      // static int trap_150_state = 0;
      sector =
          12 * (normalize_angle(el_angle + _PI_6) / _2PI); // adding PI/6 to align with other modes
      // centering the voltages around either
      // modulation_centered_ == true > driver.volage_limit/2
      // modulation_centered_ == false > or Adaptable centering, all phases drawn to 0 when uq=0
      center = modulation_centered_ ? (driver_->get_voltage_limit()) / 2 : uq;

      if (trap_150_map[sector][0] == _HIGH_IMPEDANCE) {
        Ua = center;
        Ub = trap_150_map[sector][1] * uq + center;
        Uc = trap_150_map[sector][2] * uq + center;
        driver_->set_phase_state(_HIGH_IMPEDANCE, _ACTIVE, _ACTIVE); // disable phase if possible
      } else if (trap_150_map[sector][1] == _HIGH_IMPEDANCE) {
        Ua = trap_150_map[sector][0] * uq + center;
        Ub = center;
        Uc = trap_150_map[sector][2] * uq + center;
        driver_->set_phase_state(_ACTIVE, _HIGH_IMPEDANCE, _ACTIVE); // disable phase if possible
      } else {
        Ua = trap_150_map[sector][0] * uq + center;
        Ub = trap_150_map[sector][1] * uq + center;
        Uc = center;
        driver_->set_phase_state(_ACTIVE, _ACTIVE, _HIGH_IMPEDANCE); // disable phase if possible
      }
      break;

    case detail::FocType::SINE_PWM:
      // Sinusoidal PWM modulation
      // Inverse Park + Clarke transformation

      // angle normalization in between 0 and 2pi
      // only necessary if using fast_sin and fast_cos - approximation functions
      el_angle = normalize_angle(el_angle);
      _ca = fast_cos(el_angle);
      _sa = fast_sin(el_angle);
      // Inverse park transform
      Ualpha = _ca * ud - _sa * uq; // -sin(angle) * uq;
      Ubeta = _sa * ud + _ca * uq;  //  cos(angle) * uq;

      // center = modulation_centered_ ? (driver_->get_voltage_limit())/2 : uq;
      center = driver_->get_voltage_limit() / 2;
      // Clarke transform
      Ua = Ualpha + center;
      Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta + center;
      Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta + center;

      if (!modulation_centered_) {
        float Umin = std::min(Ua, std::min(Ub, Uc));
        Ua -= Umin;
        Ub -= Umin;
        Uc -= Umin;
      }

      break;

    case detail::FocType::SPACE_VECTOR_PWM:
      // Nice video explaining the SpaceVectorModulation (SVPWM) algorithm
      // https://www.youtube.com/watch?v=QMSWUMEAejg

      // the algorithm goes
      // 1) Ualpha, Ubeta
      // 2) Uout = sqrt(Ualpha^2 + Ubeta^2)
      // 3) el_angle = atan2(Ubeta, Ualpha)
      //
      // equivalent to 2)  because the magnitude does not change is:
      // Uout = sqrt(ud^2 + uq^2)
      // equivalent to 3) is
      // el_angle = el_angle + atan2(uq,ud)

      float Uout;
      // a bit of optitmisation
      if (ud) { // only if ud and uq set
        // fast_sqrt is an approx of sqrt (3-4% error)
        Uout = fast_sqrt(ud * ud + uq * uq) / driver_->get_voltage_limit();
        // angle normalisation in between 0 and 2pi
        // only necessary if using fast_sin and fast_cos - approximation functions
        el_angle = normalize_angle(el_angle + atan2(uq, ud));
      } else { // only uq available - no need for atan2 and sqrt
        Uout = uq / driver_->get_voltage_limit();
        // angle normalisation in between 0 and 2pi
        // only necessary if using fast_sin and fast_cos - approximation functions
        el_angle = normalize_angle(el_angle + M_PI_2);
      }
      // find the sector we are in currently
      sector = floor(el_angle / _PI_3) + 1;
      // calculate the duty cycles
      float T1 = _SQRT3 * fast_sin(sector * _PI_3 - el_angle) * Uout;
      float T2 = _SQRT3 * fast_sin(el_angle - (sector - 1.0f) * _PI_3) * Uout;
      // two versions possible
      float T0 = 0; // pulled to 0 - better for low power supply voltage
      if (modulation_centered_) {
        T0 = 1 - T1 - T2; // modulation_centered_ around driver_->get_voltage_limit()/2
      }

      // calculate the duty cycles(times)
      float Ta, Tb, Tc;
      switch (sector) {
      case 1:
        Ta = T1 + T2 + T0 / 2;
        Tb = T2 + T0 / 2;
        Tc = T0 / 2;
        break;
      case 2:
        Ta = T1 + T0 / 2;
        Tb = T1 + T2 + T0 / 2;
        Tc = T0 / 2;
        break;
      case 3:
        Ta = T0 / 2;
        Tb = T1 + T2 + T0 / 2;
        Tc = T2 + T0 / 2;
        break;
      case 4:
        Ta = T0 / 2;
        Tb = T1 + T0 / 2;
        Tc = T1 + T2 + T0 / 2;
        break;
      case 5:
        Ta = T2 + T0 / 2;
        Tb = T0 / 2;
        Tc = T1 + T2 + T0 / 2;
        break;
      case 6:
        Ta = T1 + T2 + T0 / 2;
        Tb = T0 / 2;
        Tc = T1 + T0 / 2;
        break;
      default:
        // possible error state
        Ta = 0;
        Tb = 0;
        Tc = 0;
      }

      // calculate the phase voltages and center
      Ua = Ta * driver_->get_voltage_limit();
      Ub = Tb * driver_->get_voltage_limit();
      Uc = Tc * driver_->get_voltage_limit();
      break;
    }

    // set the voltages in driver
    driver_->set_voltage(Ua, Ub, Uc);
  }

  /**
   * @brief Return the shaft angle, in radians.
   * @return Motor shaft angle in radians.
   */
  float get_shaft_angle() {
    // if no sensor linked return previous value ( for open loop )
    if (!sensor_)
      return shaft_angle_;
    auto sensed = angle_filter_ ? angle_filter_(sensor_->get_radians()) : sensor_->get_radians();
    return (float)sensor_direction_ * sensed - sensor_offset_;
  }

  /**
   * @brief Return the shaft velocity, in radians per second (rad/s).
   * @return Motor shaft velocity (rad/s).
   */
  float get_shaft_velocity() {
    // if no sensor linked return previous value ( for open loop )
    if (!sensor_)
      return shaft_velocity_;
    auto sensed = velocity_filter_ ? velocity_filter_(sensor_->get_rpm()) : sensor_->get_rpm();
    return (float)sensor_direction_ * sensed * RPM_TO_RADS;
  }

  /**
   * @brief Get the electrical angle of the motor - using the mechanical
   *        angle, the number of pole pairs, and the zero electrical angle.
   * @return The electrical angle of the motor (in radians)
   */
  float get_electrical_angle() {
    if (!sensor_)
      return electrical_angle_;
    float el_angle =
        (float)sensor_direction_ * (float)num_pole_pairs_ * sensor_->get_mechanical_radians() -
        zero_electrical_angle_;
    return normalize_angle(el_angle);
  }

  /**
   * @brief Main FOC loop for implementing the torque control, based on the
   *        configured detail::TorqueControlType.
   * @note Only detail::TorqueControlType::VOLTAGE is supported right now, because the
   *       other types require current sense.
   */
  void loop_foc() {
    if (!enabled_) {
      return;
    }

    // if open-loop do nothing
    if (motion_control_type_ == detail::MotionControlType::ANGLE_OPENLOOP ||
        motion_control_type_ == detail::MotionControlType::VELOCITY_OPENLOOP) {
      return;
    }

    // This function will not have numerical issues because it uses Sensor::getMechanicalAngle()
    // which is in range 0-2PI
    electrical_angle_ = get_electrical_angle();
    switch (torque_control_type_) {
    case detail::TorqueControlType::VOLTAGE:
      // no need to do anything really
      break;
    case detail::TorqueControlType::DC_CURRENT:
      if (!current_sense_)
        return;
      // read overall current magnitude
      current_.q = current_sense_->get_dc_current(electrical_angle_);
      // filter the value values
      current_.q = q_current_filter_ ? q_current_filter_(current_.q) : current_.q;
      // calculate the phase voltage
      voltage_.q = pid_current_q_(target_current_ - current_.q);
      voltage_.d = 0;
      break;
    case detail::TorqueControlType::FOC_CURRENT:
      if (!current_sense_)
        return;
      // read dq currents
      current_ = current_sense_->get_foc_currents(electrical_angle_);
      // filter values
      current_.q = q_current_filter_ ? q_current_filter_(current_.q) : current_.q;
      current_.d = d_current_filter_ ? d_current_filter_(current_.d) : current_.d;
      // calculate the phase voltages
      voltage_.q = pid_current_q_(target_current_ - current_.q);
      voltage_.d = pid_current_d_(-current_.d);
      break;
    default:
      break;
    }

    // set the phase voltage - FOC heart function :)
    set_phase_voltage(voltage_.q, voltage_.d, electrical_angle_);
  }

  /**
   * @brief Main motion control loop implementing the closed-loop and
   *        open-loop angle & velocity control.
   * @param new_target The new target for the configured detail::MotionControlType.
   * @note Units are based on the detail::MotionControlType; radians if it's
   *       detail::MotionControlType::ANGLE or detail::MotionControlType::ANGLE_OPENLOOP,
   *       radians/second if it's detail::MotionControlType::VELOCITY or
   *       detail::MotionControlType::VELOCITY_OPENLOOP, Nm if it's
   *       detail::MotionControlType::TORQUE.
   */
  void move(float new_target) {
    // if disabled do nothing
    if (!enabled_)
      return;

    // shaft angle/velocity need the update() to be called first
    // get shaft angle
    // TODO sensor precision: the shaft_angle actually stores the complete
    //                        position, including full rotations, as a float For
    //                        this reason it is NOT precise when the angles
    //                        become large. Additionally, the way LPF works on
    //                        angle is a precision issue, and the angle-LPF is a
    //                        problem when switching to a 2-component
    //                        representation.
    if (motion_control_type_ != detail::MotionControlType::ANGLE_OPENLOOP &&
        motion_control_type_ != detail::MotionControlType::VELOCITY_OPENLOOP) {
      shaft_angle_ = get_shaft_angle();
    }

    // get angular velocity TODO the velocity reading probably also shouldn't
    // happen in open loop modes?

    shaft_velocity_ = get_shaft_velocity();

    // set internal target variable
    target_ = new_target;

    // calculate the back-emf voltage if KV_rating available U_bemf = vel*(1/KV)
    if (kv_rating_)
      bemf_voltage_ = shaft_velocity_ / kv_rating_ / RPM_TO_RADS;
    // estimate the motor current if phase reistance available and current_sense not available
    if (!current_sense_ && phase_resistance_)
      current_.q = (voltage_.q - bemf_voltage_) / phase_resistance_;

    // upgrade the current based voltage limit
    switch (motion_control_type_) {
    case detail::MotionControlType::TORQUE:
      if (torque_control_type_ == detail::TorqueControlType::VOLTAGE) { // if voltage torque control
        if (!phase_resistance_)
          voltage_.q = target_;
        else
          voltage_.q = target_ * phase_resistance_ + bemf_voltage_;
        voltage_.q = std::clamp(voltage_.q, -voltage_limit_, voltage_limit_);
        voltage_.d = 0;
      } else {
        target_current_ = target_; // if current/foc_current torque control
      }
      break;
    case detail::MotionControlType::ANGLE:
      // TODO sensor precision: this calculation is not numerically precise. The
      //                        target value cannot express precise positions
      //                        when the angles are large. This results in not
      //                        being able to command small changes at high
      //                        position values. to solve this, the delta-angle
      //                        has to be calculated in a numerically precise
      //                        way.
      // angle set point
      target_shaft_angle_ = target_;
      // calculate velocity set point
      target_shaft_velocity_ = pid_angle_(target_shaft_angle_ - shaft_angle_);
      // calculate the torque command - sensor precision: this calculation is ok, but based on bad
      // value from previous calculation
      target_current_ =
          pid_velocity_(target_shaft_velocity_ - shaft_velocity_); // if voltage torque control
      // if torque controlled through voltage
      if (torque_control_type_ == detail::TorqueControlType::VOLTAGE) {
        // use voltage if phase-resistance not provided
        if (!phase_resistance_)
          voltage_.q = target_current_;
        else
          voltage_.q = std::clamp(target_current_ * phase_resistance_ + bemf_voltage_,
                                  -voltage_limit_, voltage_limit_);
        if (!phase_inductance_)
          voltage_.d = 0;
        else
          voltage_.d =
              std::clamp(-target_current_ * num_pole_pairs_ * phase_inductance_ * shaft_velocity_,
                         -voltage_limit_, voltage_limit_);
      }
      break;
    case detail::MotionControlType::VELOCITY:
      // velocity set point - sensor precision: this calculation is numerically precise.
      target_shaft_velocity_ = target_;
      // calculate the torque command
      target_current_ = pid_velocity_(target_shaft_velocity_ -
                                      shaft_velocity_); // if current/foc_current torque control
      // if torque controlled through voltage control
      if (torque_control_type_ == detail::TorqueControlType::VOLTAGE) {
        // use voltage if phase-resistance not provided
        if (!phase_resistance_)
          voltage_.q = target_current_;
        else
          voltage_.q = std::clamp(target_current_ * phase_resistance_ + bemf_voltage_,
                                  -voltage_limit_, voltage_limit_);
        if (!phase_inductance_)
          voltage_.d = 0;
        else
          voltage_.d =
              std::clamp(-target_current_ * num_pole_pairs_ * phase_inductance_ * shaft_velocity_,
                         -voltage_limit_, voltage_limit_);
      }
      break;
    case detail::MotionControlType::VELOCITY_OPENLOOP:
      // velocity control in open loop - sensor precision: this calculation is numerically precise.
      target_shaft_velocity_ = target_;
      voltage_.q =
          velocity_openloop(target_shaft_velocity_); // returns the voltage that is set to the motor
      voltage_.d = 0;
      break;
    case detail::MotionControlType::ANGLE_OPENLOOP:
      // angle control in open loop -
      // TODO sensor precision: this calculation NOT numerically precise, and
      //                        subject to the same problems in small set-point
      //                        changes at high angles as the closed loop
      //                        version.
      target_shaft_angle_ = target_;
      voltage_.q =
          angle_openloop(target_shaft_angle_); // returns the voltage that is set to the motor
      voltage_.d = 0;
      break;
    }
  }

protected:
  void init() { status_ = Status::UNCALIBRATED; }

  void init_foc(float zero_electric_offset = 0,
                detail::SensorDirection sensor_direction = detail::SensorDirection::CLOCKWISE) {
    logger_.info("Init FOC");
    status_ = Status::CALIBRATING;
    // align motor with sensor - necessary for encoders
    if (zero_electric_offset) {
      logger_.info("Updating electrical zero angle and direction: {}, {}", zero_electric_offset,
                   (int)sensor_direction);
      zero_electrical_angle_ = zero_electric_offset;
      sensor_direction_ = sensor_direction;
    }
    // align sensor and motor
    bool success = align_sensor();
    shaft_angle_ = get_shaft_angle();

    if (current_sense_) {
      success &= align_current_sense();
    }

    status_ = success ? Status::READY : Status::FAILED_CALIBRATION;
    logger_.debug("Init FOC completed: {}", success);
  }

  bool align_sensor() {
    using namespace std::chrono_literals;
    int exit_flag = 1; // success
    logger_.info("Aligning sensor");

    // check if sensor needs zero search
    if (sensor_->needs_zero_search())
      exit_flag = search_absolute_zero();
    // stop init if not found index
    if (!exit_flag)
      return exit_flag;

    // if unknown natural direction
    if (sensor_direction_ == detail::SensorDirection::UNKNOWN) {

      // find natural direction
      // move one electrical revolution forward
      for (int i = 0; i <= 500; i++) {
        float angle = _3PI_2 + _2PI * i / 500.0f;
        set_phase_voltage(voltage_sensor_align_, 0, angle);
        std::this_thread::sleep_for(1ms * 2);
      }
      // take and angle in the middle
      float mid_angle = sensor_->get_radians();
      // move one electrical revolution backwards
      for (int i = 500; i >= 0; i--) {
        float angle = _3PI_2 + _2PI * i / 500.0f;
        set_phase_voltage(voltage_sensor_align_, 0, angle);
        std::this_thread::sleep_for(1ms * 2);
      }
      float end_angle = sensor_->get_radians();
      set_phase_voltage(0, 0, 0);
      std::this_thread::sleep_for(1ms * 200);
      // determine the direction the sensor moved
      if (mid_angle == end_angle) {
        logger_.warn("Failed to notice movement when trying to find natural direction.");
        return 0; // failed calibration
      } else if (mid_angle < end_angle) {
        logger_.info("sensor direction: detail::SensorDirection::COUNTER_CLOCKWISE");
        sensor_direction_ = detail::SensorDirection::COUNTER_CLOCKWISE;
      } else {
        logger_.info("sensor direction: detail::SensorDirection::CLOCKWISE");
        sensor_direction_ = detail::SensorDirection::CLOCKWISE;
      }
      // check pole pair number
      float moved = std::abs(mid_angle - end_angle);
      if (std::abs(moved * num_pole_pairs_ - _2PI) >
          0.5f) { // 0.5f is arbitrary number it can be lower or higher!
        logger_.debug("PP check: fail - estimated pp: {:.3f}", _2PI / moved);
      } else
        logger_.debug("PP check: OK!");

    } else
      logger_.debug("Skip dir calib.");

    // zero electric angle not known
    if (!zero_electrical_angle_) {
      logger_.info("Calibrating electrical angle.");
      // align the electrical phases of the motor and sensor
      // set angle -90(270 = 3PI/2) degrees
      set_phase_voltage(voltage_sensor_align_, 0, _3PI_2);
      std::this_thread::sleep_for(1ms * 700);
      // get the current zero electric angle
      zero_electrical_angle_ = 0;
      zero_electrical_angle_ = get_electrical_angle();
      logger_.info("Got electrical angle: {}", zero_electrical_angle_);
      // zero_electrical_angle_ =
      // normalize_angle(get_electrical_angle(sensor_direction_*sensor_->get_radians(),
      // num_pole_pairs_));
      std::this_thread::sleep_for(1ms * 20);
      // stop everything
      set_phase_voltage(0, 0, 0);
      std::this_thread::sleep_for(1ms * 200);
    } else
      logger_.debug("Skip offset calib.");
    return exit_flag;
  }

  bool align_current_sense() {
    int exit_flag = 1; // success

    logger_.info("Aligning current sense");

    // align current sense and the driver
    exit_flag = current_sense_->driver_align(voltage_sensor_align_);
    if (!exit_flag) {
      // error in current sense - phase either not measured or bad connection
      logger_.debug("Align error!");
      exit_flag = 0;
    } else {
      // output the alignment status flag
      logger_.debug("Success: {}", exit_flag);
    }

    return exit_flag > 0;
  }

  bool search_absolute_zero() {
    // sensor precision: this is all ok, as the search happens near the 0-angle, where the precision
    //                    of float is sufficient.
    logger_.info("Index search...");
    // search the absolute zero with small velocity
    float limit_vel = velocity_limit_;
    float limit_volt = voltage_limit_;
    velocity_limit_ = velocity_index_search_;
    voltage_limit_ = voltage_sensor_align_;
    shaft_angle_ = 0;
    while (sensor_->needs_zero_search() && shaft_angle_ < _2PI) {
      angle_openloop(1.5f * _2PI);
    }
    // disable motor
    set_phase_voltage(0, 0, 0);
    // reinit the limits
    velocity_limit_ = limit_vel;
    voltage_limit_ = limit_volt;
    // check if the zero found
    return !sensor_->needs_zero_search();
  }

  // Function (iterative) generating open loop movement for target velocity
  // - target_shaft_velocity_ - rad/s
  // it uses voltage_limit_ variable
  float velocity_openloop(float target) {
    static auto openloop_timestamp = std::chrono::high_resolution_clock::now();
    // get current timestamp
    auto now = std::chrono::high_resolution_clock::now();
    // calculate the sample time from last call
    float Ts = std::chrono::duration<float>(now - openloop_timestamp).count();
    // ensure that the sample time is not too small or too big
    if (Ts <= 0 || Ts > 0.5f)
      Ts = 1e-3f;
    // save timestamp for next call
    openloop_timestamp = now;

    // calculate the necessary angle to achieve target velocity
    shaft_angle_ = normalize_angle(shaft_angle_ + target_shaft_velocity_ * Ts);
    // for display purposes
    shaft_velocity_ = target_shaft_velocity_;

    // use voltage limit or current limit
    float Uq = voltage_limit_;
    if (phase_resistance_) {
      Uq = std::clamp(current_limit_ * phase_resistance_ + std::abs(bemf_voltage_), -voltage_limit_,
                      voltage_limit_);
      // recalculate the current
      current_.q = (Uq - std::abs(bemf_voltage_)) / phase_resistance_;
    }
    // set the maximal allowed voltage (voltage_limit_) with the necessary angle
    set_phase_voltage(Uq, 0, calc_electrical_angle(shaft_angle_, num_pole_pairs_));

    return Uq;
  }

  // Function (iterative) generating open loop movement towards the target angle
  // - target_shaft_angle_ - rad
  // it uses voltage_limit_ and velocity_limit_ variables
  float angle_openloop(float target) {
    static auto openloop_timestamp = std::chrono::high_resolution_clock::now();
    // get current timestamp
    auto now = std::chrono::high_resolution_clock::now();
    // calculate the sample time from last call
    float Ts = std::chrono::duration<float>(now - openloop_timestamp).count();
    // quick fix for strange cases (micros overflow + timestamp not defined)
    if (Ts <= 0 || Ts > 0.5f)
      Ts = 1e-3f;
    // save timestamp for next call
    openloop_timestamp = now;

    // calculate the necessary angle to move from current position towards target angle
    // with maximal velocity (velocity_limit_)
    // TODO sensor precision: this calculation is not numerically precise. The
    // angle can grow to the point where small position changes are no longer
    // captured by the precision of floats when the total position is large.
    if (abs(target_shaft_angle_ - shaft_angle_) > abs(velocity_limit_ * Ts)) {
      shaft_angle_ += sgn(target_shaft_angle_ - shaft_angle_) * abs(velocity_limit_) * Ts;
      shaft_velocity_ = velocity_limit_;
    } else {
      shaft_angle_ = target_shaft_angle_;
      shaft_velocity_ = 0;
    }

    // use voltage limit or current limit
    float Uq = voltage_limit_;
    if (phase_resistance_) {
      Uq = std::clamp(current_limit_ * phase_resistance_ + std::abs(bemf_voltage_), -voltage_limit_,
                      voltage_limit_);
      // recalculate the current
      current_.q = (Uq - std::abs(bemf_voltage_)) / phase_resistance_;
    }
    // set the maximal allowed voltage (voltage_limit_) with the necessary angle
    // sensor precision: this calculation is OK due to the normalisation
    set_phase_voltage(Uq, 0, calc_electrical_angle(normalize_angle(shaft_angle_), num_pole_pairs_));

    return Uq;
  }

  enum class Status {
    UNINITIALIZED,
    INITIALIZING,
    UNCALIBRATED,
    CALIBRATING,
    READY,
    ERROR,
    FAILED_CALIBRATION,
    FAILED_INITIALIZATION
  };

  // state variables:
  float target_{0};           // current target value (compared to what depends on control setting)
  float shaft_angle_{0};      // current motor angle
  float electrical_angle_{0}; // current electrical angle
  float shaft_velocity_{0};   // current motor velocity
  float target_current_{0};   // q current
  float target_shaft_velocity_{0};
  float target_shaft_angle_{0};
  float bemf_voltage_{0}; // estimated back EMF voltage (if provided KV)

  DqVoltage voltage_{0, 0}; //!< current d and q voltage set to the motor
  DqCurrent current_{0, 0}; //!< current d and q current measured

  // configuration parameters
  float voltage_sensor_align_;        // sensor and motor align voltage parameter
  float velocity_index_search_{1.0f}; // target velocity for index search

  // motor physical parameters
  int num_pole_pairs_;
  float phase_resistance_;
  float phase_inductance_;
  float kv_rating_;

  // limiting variables
  float voltage_limit_{0};
  float current_limit_{0};
  float velocity_limit_{0};

  // sensor related variables
  float sensor_offset_{0};
  float zero_electrical_angle_{0};
  detail::SensorDirection sensor_direction_{detail::SensorDirection::CLOCKWISE};

  detail::FocType foc_type_{detail::FocType::SPACE_VECTOR_PWM};
  detail::MotionControlType motion_control_type_{detail::MotionControlType::VELOCITY_OPENLOOP};
  detail::TorqueControlType torque_control_type_{detail::TorqueControlType::VOLTAGE};

  bool modulation_centered_{
      true}; //!< true: centered modulation around driver limit /2; false: pulled to 0

  float Ua, Ub, Uc; //!< Current phase voltages Ua,Ub and Uc set to motor
  float Ualpha,
      Ubeta; //!< Phase voltages U alpha and U beta used for inverse Park and Clarke transform

  std::shared_ptr<D> driver_;
  std::shared_ptr<S> sensor_;
  std::shared_ptr<CS> current_sense_;

  Pid pid_current_q_;
  Pid pid_current_d_;
  Pid pid_velocity_;
  Pid pid_angle_;
  filter_fn q_current_filter_{nullptr};
  filter_fn d_current_filter_{nullptr};
  filter_fn velocity_filter_{nullptr};
  filter_fn angle_filter_{nullptr};
  std::atomic<bool> enabled_{false};
  Status status_{Status::UNINITIALIZED};
  Logger logger_;
};

} // namespace espp
