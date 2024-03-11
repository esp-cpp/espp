#pragma once

#include <atomic>
#include <functional>
#include <string>

#include "interrupt.hpp"

namespace espp {
/// \brief A class to handle a button connected to a GPIO
/// \details This class uses the ESP-IDF GPIO interrupt handler to detect
///          button presses and releases. It then calls the callback function
///          with the event.
///
/// \section button_ex0 Simple Button Example (No Callback / Interrupt)
/// \snippet button_example.cpp simple button example
/// \section button_ex1 Button Example (With Callback / Interrupt)
/// \snippet button_example.cpp button example
class Button : protected Interrupt {
public:
  // Expose some types from the Interrupt class for convenience
  using Event = Interrupt::Event;             ///< The event type for the button
  using InterruptType = Interrupt::Type;      ///< The type of interrupt for the button
  using ActiveLevel = Interrupt::ActiveLevel; ///< The active level of the button

  /// \brief The configuration for the button
  struct Config {
    std::string_view name{"Button"};  ///< Name of the button
    InterruptConfig interrupt_config; ///< Configuration for the GPIO interrupt
    Task::BaseConfig task_config{};   ///< Configuration for the button task
    espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN; ///< Log level for this class
  };

  /// \brief The simple configuration for the button
  /// \details This is a simplified configuration for the button that only
  ///          requires the GPIO number and the active level of the button.
  ///          This is useful for simple buttons that don't require a custom
  ///          task or interrupt configuration and do not need to register a
  ///          callback function.
  struct SimpleConfig {
    std::string_view name{"Button"};             ///< Name of the button
    gpio_num_t gpio_num;                         ///< The GPIO number for the button
    ActiveLevel active_level = ActiveLevel::LOW; ///< The active level of the button
    bool pullup_enabled = true;                  ///< Whether to enable the internal pullup resistor
                                                 ///< for the button
    bool pulldown_enabled = false;               ///< Whether to enable the internal pulldown
                                                 ///< resistor for the button
    espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN; ///< Log level for this class
  };

  /// \brief Construct a button
  /// \param config The configuration for the button
  explicit Button(const Config &config)
      : Interrupt({.interrupts = {config.interrupt_config},
                   .event_queue_size = 10,
                   .task_config = config.task_config,
                   .log_level = config.log_level})
      , gpio_num_(config.interrupt_config.gpio_num)
      , active_level_(config.interrupt_config.active_level) {}

  /// \brief Construct a button with a simple configuration
  /// \param config The simple configuration for the button
  explicit Button(const SimpleConfig &config)
      : Interrupt(config.name, config.log_level)
      , gpio_num_(config.gpio_num)
      , active_level_(config.active_level) {
    // configure the GPIO as input with pullup / pulldown according to the config
    logger_.debug("Configuring GPIO {} as input (active level = {}) with pullup: {}, pulldown: {}",
                  gpio_num_, active_level_, config.pullup_enabled, config.pulldown_enabled);
    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.pin_bit_mask = (1ULL << gpio_num_);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = config.pullup_enabled ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = config.pulldown_enabled ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
  }

  /// \brief Whether the button is currently pressed
  /// \return True if the button is pressed, false otherwise
  bool is_pressed() const { return is_active_level(gpio_num_, active_level_); }

protected:
  // store these here for unprotected / easy access
  int gpio_num_{-1};                           ///< The GPIO number for the button
  ActiveLevel active_level_{ActiveLevel::LOW}; ///< The active level of the button
};
} // namespace espp
