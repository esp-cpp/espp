#pragma once

#include <algorithm>
#include <mutex>

#include "logger.hpp"

namespace espp {
  /**
   *  @brief Simple PID (proportional, integral, derivative) controller class
   *         with integrator clamping, output clamping, and prevention of
   *         integrator windup during output saturation. This class is
   *         thread-safe, so you can update(), clear(), and change_gains() from
   *         multiple threads if needed.
   *
   * \section pid_ex1 Basic PID Example
   * \snippet pid_example.cpp pid example
   * \section pid_ex2 Complex PID Example
   * \snippet pid_example.cpp complex pid example
   */
  class Pid {
  public:
    struct Config {
      float kp; /**< Proportional gain. */
      float ki; /**< Integral gain. @note should not be pre-multiplied by the time constant. */
      float kd; /**< Derivative gain. @note should not be pre-divided by the time-constant. */
      float integrator_min; /**< Minimum value the integrator can have. Could be 0 or negative. Can have different magnitude from integrator_max for asymmetric response. */
      float integrator_max; /**< Maximum value the integrator can wind up to. */
      float output_min; /**< Limit the minimum output value. Can be a different magnitude from output max for asymmetric output behavior. */
      float output_max; /**< Limit the maximum output value. */
      float sampling_time_s; /**< Sampling time, the time between calls of the update() function. @note the PID class measures time between calls to update(), but this value is used for startup and to provide warnings. */
      espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; /**< Verbosity for the adc logger. */
    };

    /**
     * @brief Create the PID controller.
     */
    Pid(const Config& config) : logger_({.tag = "PID", .level = config.log_level}) {
      change_gains(config);
    }

    /**
     * @brief Change the gains and other configuration for the PID controller.
     * @param config Configuration struct with new gains and sampling time.
     */
    void change_gains(const Config& config) {
      std::lock_guard<std::recursive_mutex> lk(mutex_);
      kp_ = config.kp;
      ki_ = config.ki;
      kd_ = config.kd;
      integrator_min_ = config.integrator_min;
      integrator_max_ = config.integrator_max;
      output_min_ = config.output_min;
      output_max_ = config.output_max;
      if (config.sampling_time_s <= 0) {
        logger_.warn("Was not provided valid sampling time, setting it to 1e-3 seconds!");
        ts_ = 1e-3;
      } else {
        ts_ = config.sampling_time_s;
      }
      clear();
    }

    /**
     * @brief Clear the PID controller state.
     */
    void clear() {
      std::lock_guard<std::recursive_mutex> lk(mutex_);
      error_ = 0;
      previous_error_ = 0;
      integrator_ = 0;
    }

    /**
     * @brief Update the PID controller with the latest error measurement,
     *        getting the output control signal in return.
     *
     * @note Tracks invocation timing to better compute time-accurate
     *       integral/derivative signals. Should be called at or close to the
     *       frequency defined by the configured Config::sampling_time_s.
     *
     * @param error Latest error signal.
     * @return The output control signal based on the PID state and error.
     */
    float update(float error) {
      static auto prev_ts = std::chrono::high_resolution_clock::now();
      auto curr_ts = std::chrono::high_resolution_clock::now();
      float t = std::chrono::duration<float>(curr_ts - prev_ts).count();
      prev_ts = curr_ts;
      // NOTE: for ESP platform, we shouldn't be running PID on anything faster
      //       than a few KHz so check against 100KHz here.
      if (t <= 1e-5) {
        // during startup, just use the the configured sampling time
        t = ts_;
      }
      std::lock_guard<std::recursive_mutex> lk(mutex_);
      error_ = error;
      integrator_ = std::clamp(integrator_ + error_, integrator_min_, integrator_max_);
      float p = kp_ * error_;
      float i = ki_ * integrator_ * t;
      float d = kd_ * (error_ - previous_error_) / t;
      float output = p + i + d;
      // ensure we don't continue growing integrator if the output is saturated
      if (output >= output_max_ || output <= output_min_) {
        integrator_ -= error_;
      }
      // update our state for next loop
      previous_error_ = error_;
      // clamp the output and return it
      return std::clamp(output, output_min_, output_max_);
    }

  protected:
    float kp_;
    float ki_;
    float kd_;
    float integrator_min_;
    float integrator_max_;
    float output_min_;
    float output_max_;
    float error_;
    float previous_error_;
    float integrator_;
    float ts_;
    std::recursive_mutex mutex_;
    Logger logger_;
  };
}
