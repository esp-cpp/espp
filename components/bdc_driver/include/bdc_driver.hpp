#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h>

#include "base_component.hpp"

namespace espp {
/// Driver for brushed DC motors controlled by two PWM outputs.
///
/// The component uses one MCPWM operator per motor channel and is intended for
/// H-bridge style drivers that accept two PWM inputs. Positive signed speed
/// commands drive output A, negative commands drive output B, and zero disables
/// both outputs.
///
/// \section bdc_driver_example Example
/// \snippet bdc_driver_example.cpp bdc_driver example
class BdcDriver : public BaseComponent {
public:
  /// Default MCPWM timer resolution used by the driver.
  static constexpr size_t DEFAULT_TIMER_RESOLUTION_HZ = 80 * 1000 * 1000;
  /// Default PWM carrier frequency used by the driver.
  static constexpr size_t DEFAULT_PWM_FREQUENCY_HZ = 20 * 1000;

  /// Configuration for one brushed DC motor driver.
  struct Config {
    gpio_num_t gpio_a{GPIO_NUM_NC};      ///< PWM GPIO for forward/output A.
    gpio_num_t gpio_b{GPIO_NUM_NC};      ///< PWM GPIO for reverse/output B.
    gpio_num_t gpio_enable{GPIO_NUM_NC}; ///< Optional active-high enable pin.
    int group_id{0};                     ///< MCPWM group to allocate the timer and operator from.
    size_t timer_resolution_hz{
        DEFAULT_TIMER_RESOLUTION_HZ};                  ///< MCPWM timer clock resolution in Hz.
    size_t pwm_frequency_hz{DEFAULT_PWM_FREQUENCY_HZ}; ///< PWM carrier frequency in Hz.
    bool invert_output_a{false}; ///< True to invert output A at the GPIO matrix.
    bool invert_output_b{false}; ///< True to invert output B at the GPIO matrix.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Component log verbosity.
  };

  /// Construct the brushed DC motor driver.
  /// \param config Driver configuration.
  explicit BdcDriver(const Config &config);

  /// Destroy the driver and release all MCPWM resources.
  ~BdcDriver();

  /// Check whether initialization completed successfully.
  /// \return True if the MCPWM resources were created and the outputs were
  ///         initialized; false otherwise.
  bool initialized() const;

  /// Enable the motor driver output.
  /// \return True if the timer started successfully and the optional enable pin
  ///         was asserted; false otherwise.
  bool enable();

  /// Disable the motor driver output.
  /// \return True if the outputs were forced low and the timer was stopped;
  ///         false otherwise.
  bool disable();

  /// Check whether the driver is currently enabled.
  /// \return True if the timer is running; false otherwise.
  bool is_enabled() const;

  /// Set the normalized duty cycles for the two driver outputs.
  /// \param duty_a Duty cycle for output A in the range [0.0, 1.0].
  /// \param duty_b Duty cycle for output B in the range [0.0, 1.0].
  /// \return True if both outputs were updated successfully; false otherwise.
  bool set_duty(float duty_a, float duty_b);

  /// Set a signed normalized motor command.
  /// \param speed Signed speed command in the range [-1.0, 1.0]. Positive
  ///              values drive output A, negative values drive output B, and
  ///              zero disables both outputs.
  /// \return True if the outputs were updated successfully; false otherwise.
  bool set_speed(float speed);

  /// Stop the motor by disabling both PWM outputs.
  /// \return True if both outputs were set low; false otherwise.
  bool stop();

  /// Get the last commanded normalized duty cycles.
  /// \return Array containing output A then output B duty cycles in the range
  ///         [0.0, 1.0].
  std::array<float, 2> duty_cycle() const;

  /// Get the last commanded raw duty counts.
  /// \return Array containing output A then output B raw compare values.
  std::array<uint32_t, 2> raw_duty() const;

  /// Get the maximum raw duty count for this driver's timer configuration.
  /// \return Maximum raw duty count corresponding to 100% duty.
  uint32_t max_raw_duty() const;

  /// Get the configured PWM carrier frequency.
  /// \return PWM carrier frequency in Hz.
  size_t pwm_frequency_hz() const;

  /// Get the configured timer resolution.
  /// \return MCPWM timer resolution in Hz.
  size_t timer_resolution_hz() const;

  /// Get the configured MCPWM group.
  /// \return MCPWM group index used by this driver.
  int group_id() const;

  /// Get output A's GPIO.
  /// \return Output A GPIO.
  gpio_num_t gpio_a() const;

  /// Get output B's GPIO.
  /// \return Output B GPIO.
  gpio_num_t gpio_b() const;

protected:
  bool init(const Config &config);
  bool configure_enable_gpio();
  bool configure_timer_and_operator();
  bool configure_outputs(const Config &config);
  bool configure_generator(mcpwm_gen_handle_t generator, mcpwm_cmpr_handle_t comparator);
  bool apply_output(size_t index, float duty);
  void cleanup();

  gpio_num_t gpio_a_{GPIO_NUM_NC};
  gpio_num_t gpio_b_{GPIO_NUM_NC};
  gpio_num_t gpio_enable_{GPIO_NUM_NC};
  int group_id_{0};
  size_t timer_resolution_hz_{DEFAULT_TIMER_RESOLUTION_HZ};
  size_t pwm_frequency_hz_{DEFAULT_PWM_FREQUENCY_HZ};
  uint32_t period_ticks_{0};
  bool initialized_{false};
  std::atomic<bool> enabled_{false};
  mcpwm_timer_handle_t timer_{nullptr};
  mcpwm_oper_handle_t operator_{nullptr};
  std::array<mcpwm_cmpr_handle_t, 2> comparators_{};
  std::array<mcpwm_gen_handle_t, 2> generators_{};
  std::array<float, 2> duty_cycle_{0.0f, 0.0f};
  std::array<uint32_t, 2> raw_duty_{0, 0};
}; // class BdcDriver
} // namespace espp
