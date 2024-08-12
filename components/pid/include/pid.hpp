#pragma once

#include <algorithm>
#include <atomic>
#include <mutex>

#include "base_component.hpp"

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
class Pid : public BaseComponent {
public:
  struct Config {
    float kp; /**< Proportional gain. */
    float ki; /**< Integral gain. @note should not be pre-multiplied by the time constant. */
    float kd; /**< Derivative gain. @note should not be pre-divided by the time-constant. */
    float integrator_min; /**< Minimum value the integrator can wind down to. @note Operates at the
                             same scale as \p output_min and \p output_max. Could be 0 or negative.
                             Can have different magnitude from integrator_max for asymmetric
                             response. */
    float integrator_max; /**< Maximum value the integrator can wind up to. @note Operates at the
                             same scale as \p output_min and \p output_max. */
    float output_min; /**< Limit the minimum output value. Can be a different magnitude from output
                         max for asymmetric output behavior. */
    float output_max; /**< Limit the maximum output value. */
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; /**< Verbosity for the adc logger. */
  };

  /**
   * @brief Create the PID controller.
   */
  explicit Pid(const Config &config)
      : BaseComponent("PID", config.log_level) {
    change_gains(config);
  }

  /**
   * @brief Change the gains and other configuration for the PID controller.
   * @param config Configuration struct with new gains and sampling time.
   * @param reset_state Reset / clear the PID controller state.
   */
  void change_gains(const Config &config, bool reset_state = true) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    logger_.info("Updated config: {}", config);
    config_ = config;
    if (reset_state)
      clear(); // clear the state
  }

  /**
   * @brief Change the gains and other configuration for the PID controller.
   * @param config Configuration struct with new gains and sampling time.
   * @param reset_state Reset / clear the PID controller state.
   */
  void set_config(const Config &config, bool reset_state = true) {
    change_gains(config, reset_state);
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
   *       integral/derivative signals.
   *
   * @param error Latest error signal.
   * @return The output control signal based on the PID state and error.
   */
  float update(float error) {
#if defined(ESP_PLATFORM)
    auto curr_ts = esp_timer_get_time();
    if (prev_ts_ == 0) {
      prev_ts_ = curr_ts;
    }
    float t = (curr_ts - prev_ts_) / 1e6f; // convert to seconds from microseconds
#else                                      // ESP_PLATFORM
    auto curr_ts = std::chrono::high_resolution_clock::now();
    if (prev_ts_ == std::chrono::high_resolution_clock::time_point())
      prev_ts_ = curr_ts;
    float t = std::chrono::duration<float>(curr_ts - prev_ts_).count();
#endif                                     // ESP_PLATFORM
    prev_ts_ = curr_ts;
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    error_ = error;
    float integrand = config_.ki * error_ * t;
    integrator_ =
        std::clamp(integrator_ + integrand, config_.integrator_min, config_.integrator_max);
    float p = config_.kp * error_;
    float i = integrator_;
    float d = t > 0 ? config_.kd * (error_ - previous_error_) / t : 0;
    float output = p + i + d;
    // update our state for next loop
    previous_error_.store(error_);
    // ensure we don't continue growing integrator (windup) if the output is saturated
    if (output >= config_.output_max || output <= config_.output_min) {
      integrator_ = integrator_ - integrand;
    }
    // clamp the output and return it
    return std::clamp(output, config_.output_min, config_.output_max);
  }

  /**
   * @brief Update the PID controller with the latest error measurement,
   *        getting the output control signal in return.
   *
   * @note Tracks invocation timing to better compute time-accurate
   *       integral/derivative signals.
   *
   * @param error Latest error signal.
   * @return The output control signal based on the PID state and error.
   */
  float operator()(float error) { return update(error); }

  /**
   * @brief Get the current error (as of the last time update() or operator()
   *        were called)
   * @return Most recent error.
   */
  float get_error() const { return error_; }

  /**
   * @brief Get the current integrator (as of the last time update() or
   *        operator() were called)
   * @return Most recent integrator value.
   */
  float get_integrator() const { return integrator_; }

  /**
   * @brief Get the configuration for the PID (gains, etc.).
   * @return Config structure containing gains, etc.
   */
  Config get_config() const { return config_; }

protected:
  Config config_;
  std::atomic<float> error_{0};
  std::atomic<float> previous_error_{0};
  std::atomic<float> integrator_{0};
#if defined(ESP_PLATFORM)
  uint64_t prev_ts_{0};
#else
  std::chrono::high_resolution_clock::time_point prev_ts_;
#endif
  std::recursive_mutex mutex_; ///< For protecting the config
};
} // namespace espp

#include "pid_formatters.hpp"
