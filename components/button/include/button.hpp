#pragma once

#include <atomic>
#include <functional>

#include <driver/gpio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "logger.hpp"
#include "task.hpp"

namespace espp {
/// \brief A class to handle a button connected to a GPIO
/// \details This class uses the ESP-IDF GPIO interrupt handler to detect
///          button presses and releases. It then calls the callback function
///          with the event.
///
/// \section button_ex1 Button Example
/// \snippet button_example.cpp button example
class Button {
public:
  /// \brief The event for the button
  struct Event {
    uint8_t gpio_num; ///< The GPIO number of the button
    bool pressed;     ///< Whether the button is pressed
  };

  /// \brief The active level of the GPIO
  enum class ActiveLevel {
    LOW = 0,  ///< Active low
    HIGH = 1, ///< Active high
  };

  /// \brief The type of interrupt to use for the GPIO
  enum class InterruptType {
    ANY_EDGE = GPIO_INTR_ANYEDGE,      ///< Interrupt on any edge
    RISING_EDGE = GPIO_INTR_POSEDGE,   ///< Interrupt on rising edge
    FALLING_EDGE = GPIO_INTR_NEGEDGE,  ///< Interrupt on falling edge
    LOW_LEVEL = GPIO_INTR_LOW_LEVEL,   ///< Interrupt on low level
    HIGH_LEVEL = GPIO_INTR_HIGH_LEVEL, ///< Interrupt on high level
  };

  typedef std::function<void(const Event &)> event_callback_fn; ///< The callback for the event

  /// \brief The configuration for the button
  struct Config {
    int gpio_num;                                           ///< GPIO number to use for the button
    event_callback_fn callback;                             ///< Callback for the button event
    ActiveLevel active_level;                               ///< Active level of the GPIO
    InterruptType interrupt_type = InterruptType::ANY_EDGE; ///< Interrupt type to use for the GPIO
    bool pullup_enabled = false;   ///< Whether to enable the pullup resistor
    bool pulldown_enabled = false; ///< Whether to enable the pulldown resistor
    size_t task_stack_size_bytes =
        4 * 1024;       ///< Stack size for the task. @note This may need to be increased if the
                        ///< callback is doing a lot of work (esp. string manipulation, calling many
                        ///< functions, etc.)
    size_t priority{0}; ///< Priority of the button task, 0 is lowest priority on ESP / FreeRTOS.
    int core_id{-1};    ///< Core ID of the button task, -1 means it is not pinned to any core.
    espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN; ///< Log level for this class
  };

  /// \brief Construct a button
  /// \param config The configuration for the button
  explicit Button(const Config &config)
      : gpio_num_(config.gpio_num), callback_(config.callback), active_level_(config.active_level),
        logger_({.tag = "Button", .level = config.log_level}) {
    // make the event queue
    event_queue_ = xQueueCreate(10, sizeof(EventData));

    // configure the GPIO for an interrupt
    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.intr_type = static_cast<gpio_int_type_t>(config.interrupt_type);
    io_conf.pin_bit_mask = 1ULL << gpio_num_;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = config.pullup_enabled ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = config.pulldown_enabled ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // set the initial state of the button
    update();

    // install the isr handler
    handler_args_ = {
        .gpio_num = gpio_num_,
        .event_queue = event_queue_,
    };
    static bool isr_service_installed = false;
    if (!isr_service_installed) {
      gpio_install_isr_service(0);
      isr_service_installed = true;
    }
    gpio_isr_handler_add(static_cast<gpio_num_t>(gpio_num_), isr_handler, &handler_args_);

    // make the task
    task_ = espp::Task::make_unique(
        espp::Task::Config{.name = "Button",
                           .callback = std::bind(&Button::task_callback, this,
                                                 std::placeholders::_1, std::placeholders::_2),
                           .stack_size_bytes = config.task_stack_size_bytes,
                           .priority = config.priority,
                           .core_id = config.core_id});
    task_->start();
  }

  /// \brief Destroy the button
  ~Button() {
    // remove the isr handler
    gpio_isr_handler_remove(static_cast<gpio_num_t>(gpio_num_));
    // send to the event queue to wake up the task
    EventData event_data = {};
    xQueueSend(event_queue_, &event_data, 0);
    // stop the task
    task_->stop();
    // delete the event queue
    vQueueDelete(event_queue_);
  }

  /// \brief Whether the button is currently pressed
  /// \return True if the button is pressed, false otherwise
  bool is_pressed() const { return pressed_; }

protected:
  struct HandlerArgs {
    int gpio_num;
    QueueHandle_t event_queue;
  };

  struct EventData {
    int gpio_num;
  };

  bool update() {
    auto new_state =
        gpio_get_level(static_cast<gpio_num_t>(gpio_num_)) == static_cast<int>(active_level_);
    logger_.debug("Button new state: {}", new_state);
    if (new_state != pressed_) {
      pressed_ = new_state;
      return true;
    }
    return false;
  }

  static void isr_handler(void *arg) {
    HandlerArgs *handler_args = static_cast<HandlerArgs *>(arg);
    EventData event_data = {
        .gpio_num = handler_args->gpio_num,
    };
    xQueueSendFromISR(handler_args->event_queue, &event_data, nullptr);
  }

  bool task_callback(std::mutex &m, std::condition_variable &cv) {
    EventData event_data;
    if (xQueueReceive(event_queue_, &event_data, portMAX_DELAY)) {
      if (event_data.gpio_num != gpio_num_) {
        logger_.error("Received event for wrong GPIO");
        return false;
      }
      bool updated = update();
      logger_.debug("ISR Notify, button state: {}, was updated: {}", pressed_, updated);
      Event event = {
          .gpio_num = static_cast<uint8_t>(gpio_num_),
          .pressed = pressed_,
      };
      if (callback_) {
        callback_(event);
      }
    }
    // we don't want to stop the task, so return false
    return false;
  }

  int gpio_num_;
  event_callback_fn callback_;
  ActiveLevel active_level_;
  QueueHandle_t event_queue_;
  HandlerArgs handler_args_;
  std::atomic<bool> pressed_{false};
  std::unique_ptr<espp::Task> task_;
  espp::Logger logger_;
};
} // namespace espp
