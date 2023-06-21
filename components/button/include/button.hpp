#pragma once

#include <atomic>

#include <driver/gpio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "event_manager.hpp"
#include "logger.hpp"
#include "serialization.hpp"
#include "task.hpp"

namespace espp {
/// \brief A class to handle a button connected to a GPIO
/// \details This class uses the ESP-IDF GPIO interrupt handler to detect
///          button presses and releases. It then publishes events to the
///          event manager with the topic specified in the config. The events
///          are "pressed" and "released".
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

  /// \brief The configuration for the button
  struct Config {
    int gpio_num;      ///< GPIO number to use for the button
    std::string topic; ///< Topic to publish events to via the event manager, e.g. "button/state"
    std::string
        component_name; ///< Name of the component for logging and event manager, e.g. "button"
    ActiveLevel active_level;                               ///< Active level of the GPIO
    InterruptType interrupt_type = InterruptType::ANY_EDGE; ///< Interrupt type to use for the GPIO
    bool pullup_enabled = false;   ///< Whether to enable the pullup resistor
    bool pulldown_enabled = false; ///< Whether to enable the pulldown resistor
    espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN; ///< Log level for this class
  };

  /// \brief Construct a button
  /// \param config The configuration for the button
  explicit Button(const Config &config)
      : gpio_num_(config.gpio_num), topic_(config.topic),
        logger_({.tag = config.component_name, .level = config.log_level}) {
    // make the event queue
    event_queue_ = xQueueCreate(10, sizeof(EventData));

    // register with the event manager that we want to publish events
    espp::EventManager::get().add_publisher(topic_, config.component_name);

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
    pressed_ =
        gpio_get_level(static_cast<gpio_num_t>(gpio_num_)) == static_cast<int>(config.active_level);

    // install the isr handler
    handler_args_ = {
        .gpio_num = gpio_num_,
        .event_queue = event_queue_,
    };
    gpio_install_isr_service(0);
    gpio_isr_handler_add(static_cast<gpio_num_t>(gpio_num_), isr_handler, &handler_args_);

    // make the task
    task_ = espp::Task::make_unique(
        espp::Task::Config{.name = "Button",
                           .callback = std::bind(&Button::task_callback, this,
                                                 std::placeholders::_1, std::placeholders::_2),
                           .stack_size_bytes = 4 * 1024});
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
      pressed_ = !pressed_;
      logger_.debug("Button state: {}", pressed_);
      Event event = {
          .gpio_num = static_cast<uint8_t>(gpio_num_),
          .pressed = pressed_,
      };
      std::vector<uint8_t> buffer;
      auto bytes_written = espp::serialize(event, buffer);
      if (bytes_written) {
        std::string data(buffer.begin(), buffer.end());
        espp::EventManager::get().publish(topic_, data);
      } else {
        logger_.error("Failed to serialize event data");
      }
    }
    // we don't want to stop the task, so return false
    return false;
  }

  int gpio_num_;
  QueueHandle_t event_queue_;
  HandlerArgs handler_args_;
  std::atomic<bool> pressed_{false};
  std::string topic_;
  std::unique_ptr<espp::Task> task_;
  espp::Logger logger_;
};
} // namespace espp
