#pragma once

#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <driver/gpio.h>
#include <driver/gpio_filter.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "base_component.hpp"
#include "run_on_core.hpp"
#include "task.hpp"

namespace espp {
/// \brief A class to handle a GPIO interrupt
/// \details This class uses the ESP-IDF GPIO interrupt handler to detect
///          GPIO interrupts. It then calls the callback function with the event.
///          It can handle multiple GPIO interrupts and call the appropriate callback
///          for each GPIO interrupt.
///
///          The class uses a FreeRTOS queue to handle the events. The queue is
///          created in the constructor and deleted in the destructor. The queue
///          is used to wake up the task when an interrupt event occurs. The task
///          then calls the appropriate callback for the interrupt. Since all the
///          GPIO interrupts are handled by the same task, all the callbacks are
///          called from the same task. This means that the callbacks should be
///          fast and not block for long periods of time, otherwise the other
///          interrupts will be delayed.
///
///          Regardless of the callback speed, some interrupts could still be
///          missed if they happen too quickly. For this reason, the queue size
///          can be set in the configuration (default is 10). If the queue is full,
///          then the interrupt event will be missed. If you are expecting a lot
///          of interrupts to happen quickly, then you should increase the queue
///          size.
///
///          Another way to handle the situation where you have many interrupts
///          and would like to separate out the processing by priority is to
///          have different Interrupt objects for the different priority levels,
///          and assign the interrupt pins/callbacks to the objects according to
///          the priority levels that you want. This will ensure that interrupts
///          at different priority levels / in different objects do not starve
///          the each other, while ensuring that the interupts are still
///          processed in an orderly fashion.
///
///          If CONFIG_GPIO_CTRL_FUNC_IN_IRAM is enabled, then the ISR handler
///          will be placed in IRAM. In this condition, the interrupt class' ISR
///          handler will automatically disable the interrupt associated with
///          that GPIO within the ISR handler.
///
///          If CONFIG_GPIO_CTRL_FUNC_IN_IRAM is enabled and the PinConfig has
///          auto_reenable set to false, then the interrupt will not be
///          reenabled automatically. This is because the ISR handler will
///          disable the interrupt, and it will not be reenabled until the user
///          reenables it. This use-case is recommended for ACTIVE_LOW or
///          ACTIVE_HIGH interrupts which may need some other action to clear
///          the interrupt condition and prevent the ISR from being triggered
///          continuously. In this case, simply re-enable the interrupt when you
///          have cleared the interrupt condition if you want to be able to
///          respond to it again.
///
/// \section interrupt_ex0 Interrupt Example
/// \snippet interrupt_example.cpp interrupt example
class Interrupt : public BaseComponent {
public:
  /// \brief The event for the interrupt
  struct Event {
    uint8_t gpio_num; ///< The GPIO number of the interrupt
    bool active;      ///< Whether the interrupt is active or not (based on the active level)
  };

  /// \brief The active level of the GPIO
  enum class ActiveLevel {
    LOW = 0,  ///< Active low
    HIGH = 1, ///< Active high
  };

  /// \brief The type of interrupt to use for the GPIO
  enum class Type {
    ANY_EDGE = GPIO_INTR_ANYEDGE,      ///< Interrupt on any edge
    RISING_EDGE = GPIO_INTR_POSEDGE,   ///< Interrupt on rising edge
    FALLING_EDGE = GPIO_INTR_NEGEDGE,  ///< Interrupt on falling edge
    LOW_LEVEL = GPIO_INTR_LOW_LEVEL,   ///< Interrupt on low level
    HIGH_LEVEL = GPIO_INTR_HIGH_LEVEL, ///< Interrupt on high level
  };

  typedef std::function<void(const Event &)> event_callback_fn; ///< The callback for the event

  enum class FilterType {
    NONE,              ///< No filter
    PIN_GLITCH_FILTER, ///< The pin glitch filter
    FLEX_GLITCH_FILTER ///< The flex glitch filter
  };

  /// \brief The configuration for the filter
  /// \note This is only supported on some chips (-C and -S series chips) and is
  ///       only enabled if CONFIG_SOC_GPIO_FLEX_GLITCH_FILTER_NUM > 0.
  /// \note This filter config is only supported by the flex_glitch_filter. The
  ///       pin_glitch_filter is not-configurable.
  /// \details This is used to configure the GPIO flex glitch filter The filter
  ///         is used to filter out glitches on the GPIO whose pulses are
  ///         shorter than window_threshold_ns within the window_width_ns
  ///         sampling window.
  ///
  struct FilterConfig {
    uint32_t window_width_ns{
        10000}; ///< The width of the sampling window in nanoseconds. Default is 10us
    uint32_t window_threshold_ns{5000}; ///< The threshold for the sampling window in
                                        /// nanoseconds. If the width of the pulse is
                                        /// less than this, it is filtered out. Default
                                        /// is 5us
  };

  /// \brief The configuration for an interrupt on a GPIO
  /// \details This is used to configure the GPIO interrupt
  struct PinConfig {
    int gpio_num;                         ///< GPIO number to for this interrupt
    event_callback_fn callback;           ///< Callback for the interrupt event
    bool auto_reenable = true;            ///< Whether to auto reenable the interrupt
                                          ///   after it is triggered. If false, the
                                          ///   interrupt will need to be reenabled in the
                                          ///   callback or some other codepath. If true,
                                          ///   the interrupt will be reenabled
                                          ///   automatically before the callback is
                                          ///   called.
    ActiveLevel active_level;             ///< Active level of the GPIO
    Type interrupt_type = Type::ANY_EDGE; ///< Interrupt type to use for the GPIO
    bool pullup_enabled = false;          ///< Whether to enable the pullup resistor
    bool pulldown_enabled = false;        ///< Whether to enable the pulldown resistor
    FilterType filter_type =
        FilterType::NONE;         ///< The type of filter to use. If set to FLEX_GLITCH_FILTER, the
                                  ///< filter_config should be set.
    FilterConfig filter_config{}; ///< The configuration for the filter. This is only used if
                                  ///< filter_type is set to FLEX_GLITCH_FILTER
  };

  /// \brief The configuration for the interrupt
  struct Config {
    int isr_core_id = -1; ///< The core to install the ISR service on. If -1, then the ISR
                          ///        service is installed on the core that this constructor is
                          ///        called on. If 0 or 1, then the ISR service is installed on
                          ///        the specified core. If the ISR service is already installed,
                          ///        then this function does nothing. If the core_id is invalid,
                          ///        then an error is logged and the ISR service is not installed.
    std::vector<PinConfig> interrupts; ///< The configuration for the interrupts
    size_t event_queue_size = 10;      ///< The size of the event queue
    Task::BaseConfig task_config;      ///< The configuration for the task
    espp::Logger::Verbosity log_level =
        espp::Logger::Verbosity::WARN; ///< The log level for the interrupt
  };

  /// \brief Constructor
  /// \param config The configuration for the interrupt
  explicit Interrupt(const Config &config)
      : BaseComponent("Interrupt", config.log_level)
      , queue_(xQueueCreate(config.event_queue_size, sizeof(EventData)))
      , min_queue_size_(config.event_queue_size)
      , interrupts_(config.interrupts) {
    // create the event queue
    if (!queue_) {
      logger_.error("Failed to create event queue");
      return;
    }
    // install the ISR service
    install_isr_service(config.isr_core_id);
    // NOTE: no need for lock here, as we are in the constructor
    for (const auto &interrupt : interrupts_) {
      configure_interrupt(interrupt);
    }
    // now make and start the task
    task_ = espp::Task::make_unique({
        .callback = std::bind(&Interrupt::task_callback, this, std::placeholders::_1,
                              std::placeholders::_2, std::placeholders::_3),
        .task_config = config.task_config,
    });
    task_->start();
  }

  /// \brief Destructor
  ~Interrupt() {
    // remove the isr handlers
    remove_all();
    if (queue_) {
      // send to the event queue to wake up the task
      EventData event_data = {-1};
      xQueueSend(queue_, &event_data, 0);
      // stop the task
      task_->stop();
      // delete the queue
      vQueueDelete(queue_);
    }
  }

  /// \brief Get the minimum number of free spaces in the queue
  /// \return The minimum number of free spaces in the queue
  /// \details This will return the minimum number of free spaces in the queue
  ///          Over the lifetime of the object. This can be used to determine if
  ///          the queue size is too small for the number of interrupts that are
  ///          being received. It may also help indicate if the interrupt task
  ///          priority is too low, preventing the queue from being serviced.
  ///          Finally, it may also help to indicate if additional filtering may
  ///          be needed on the interrupt line (either using the FilterType or
  ///          with hardware filtering).
  size_t get_min_queue_size() const { return min_queue_size_; }

  /// \brief Add an interrupt to the interrupt handler
  /// \param interrupt The interrupt to add
  void add_interrupt(const PinConfig &interrupt) {
    logger_.info("Adding interrupt for GPIO {}", interrupt.gpio_num);
    std::lock_guard<std::recursive_mutex> lock(interrupt_mutex_);
    interrupts_.push_back(interrupt);
    configure_interrupt(interrupt);
  }

  /// \brief Remove all the interrupts from the interrupt handler
  /// \details This will remove all the interrupts that are currently registered
  ///          with the interrupt handler. This will also disable all the
  ///          interrupts that are currently enabled.
  void remove_all() {
    std::lock_guard<std::recursive_mutex> lock(interrupt_mutex_);
    for (const auto &interrupt : interrupts_) {
      disable_interrupt(interrupt);
    }
    // clear the interrupts vector
    interrupts_.clear();
    // delete the handler args objects
    for (const auto &args : handler_args_) {
      delete args;
    }
    // clear the handler args vector
    handler_args_.clear();
    // disable anly glitch filters if they are enabled
#if CONFIG_SOC_GPIO_SUPPORT_PIN_GLITCH_FILTER || CONFIG_SOC_GPIO_FLEX_GLITCH_FILTER_NUM > 0
    for (const auto &handle : glitch_filter_handles_) {
      // disable the glitch filters
      gpio_glitch_filter_disable(handle);
      // and delete the handle
      gpio_del_glitch_filter(handle);
    }
#endif
    // clear the glitch filter handles vector
    glitch_filter_handles_.clear();
  }

  /// \brief Remove an interrupt from the interrupt handler
  /// \param interrupt The interrupt to remove
  /// \details This will find a registered interrupt with the given GPIO number
  ///          and remove it from the list of interrupts. It will also disable
  ///          the interrupt for that GPIO number. If no interrupt is found with
  ///          the given GPIO number, then nothing is done.
  void remove_interrupt(const PinConfig &interrupt) { remove_interrupt(interrupt.gpio_num); }

  /// \brief Remove an interrupt from the interrupt handler
  /// \param gpio_num The GPIO number of the interrupt to remove
  /// \details This will find a registered interrupt with the given GPIO number
  ///          and remove it from the list of interrupts. It will also disable
  ///          the interrupt for that GPIO number. If no interrupt is found with
  ///          the given GPIO number, then nothing is done.
  void remove_interrupt(int gpio_num) {
    logger_.info("Removing interrupt for GPIO {}", gpio_num);
    // disable the interrupt
    gpio_intr_disable(static_cast<gpio_num_t>(gpio_num));
    // remove the interrupt from the list
    std::lock_guard<std::recursive_mutex> lock(interrupt_mutex_);
    auto predicate = [gpio_num](const PinConfig &interrupt) {
      return interrupt.gpio_num == gpio_num;
    };
    auto interrupt = std::find_if(interrupts_.begin(), interrupts_.end(), predicate);
    if (interrupt == interrupts_.end()) {
      logger_.error("No interrupt found for GPIO {}", gpio_num);
      return;
    }
    // erase the interrupt
    interrupts_.erase(interrupt);
    // remove the isr handler
    gpio_isr_handler_remove(static_cast<gpio_num_t>(gpio_num));
    // erase the handler args object that corresponds to the gpio_num
    auto handler_arg =
        std::find_if(handler_args_.begin(), handler_args_.end(),
                     [gpio_num](const HandlerArgs *args) { return args->gpio_num == gpio_num; });
    if (handler_arg != handler_args_.end()) {
      delete *handler_arg;
      handler_args_.erase(handler_arg);
    } else {
      logger_.error("No handler args found for GPIO {}", gpio_num);
    }
  }

  /// \brief Disable all the interrupts
  /// \details This will disable all the interrupts that are currently
  ///          registered with the interrupt handler. This will not remove the
  ///          interrupts from the list of interrupts, so they can be reenabled
  ///          later.
  void disable_all() {
    std::lock_guard<std::recursive_mutex> lock(interrupt_mutex_);
    for (const auto &interrupt : interrupts_) {
      disable_interrupt(interrupt);
    }
  }

  /// \brief Disable the interrupt for the interrupt PinConfig
  /// \param interrupt The interrupt to disable
  /// \details This will disable the interrupt for the GPIO that is specified,
  ///          regardless of whether the interrupt is in the list of interrupts
  ///          or not.
  void disable_interrupt(const PinConfig &interrupt) { disable_interrupt(interrupt.gpio_num); }

  /// \brief Disable the interrupt for the GPIO
  /// \param gpio_num The GPIO number of the interrupt to disable
  /// \details This will disable the interrupt for the GPIO that is specified,
  ///          regardless of whether the interrupt is in the list of interrupts
  ///          or not.
  void disable_interrupt(int gpio_num) {
    logger_.debug("Disabling interrupt for GPIO {}", gpio_num);
    gpio_intr_disable(static_cast<gpio_num_t>(gpio_num));
  }

  /// \brief Enable the interrupt for the GPIO
  /// \param interrupt The interrupt to enable
  /// \details This will enable the interrupt for the GPIO that is specified,
  ///          regardless of whether the interrupt is in the list of interrupts
  ///          or not.
  void enable_interrupt(const PinConfig &interrupt) { enable_interrupt(interrupt.gpio_num); }

  /// \brief Enable the interrupt for the GPIO
  /// \param gpio_num The GPIO number of the interrupt to enable
  /// \details This will enable the interrupt for the GPIO that is specified,
  ///          regardless of whether the interrupt is in the list of interrupts
  ///          or not.
  void enable_interrupt(int gpio_num) {
    logger_.debug("Enabling interrupt for GPIO {}", gpio_num);
    gpio_intr_enable(static_cast<gpio_num_t>(gpio_num));
  }

  /// \brief Get the state of the interrupt
  /// \param interrupt The interrupt to check
  /// \return Whether the interrupt is active
  /// \details This will check the raw logic level of the GPIO and return
  ///          whether the interrupt is active or not according to the active
  ///          level that was set in the configuration.
  bool is_active(const PinConfig &interrupt) const {
    return is_active_level(interrupt.gpio_num, interrupt.active_level);
  }

  /// \brief Get the state of all the interrupts
  /// \return A vector of the states of the interrupts as pairs of the GPIO
  ///         number and whether the interrupt is active
  std::vector<std::pair<int, bool>> get_active_states() {
    std::vector<std::pair<int, bool>> states;
    std::lock_guard<std::recursive_mutex> lock(interrupt_mutex_);
    for (const auto &interrupt : interrupts_) {
      bool active = is_active_level(interrupt.gpio_num, interrupt.active_level);
      states.push_back({interrupt.gpio_num, active});
    }
    return states;
  }

protected:
  // only used by subclasses which want to alter how the ISR configuration,
  // task initialization, etc. is done
  explicit Interrupt(std::string_view name, espp::Logger::Verbosity log_level)
      : BaseComponent(name, log_level) {}

  void install_isr_service(int core_id = -1) {
    if (ISR_SERVICE_INSTALLED) {
      logger_.warn("ISR service already installed, not installing again");
      return;
    }
    auto install_fn = []() -> esp_err_t { return gpio_install_isr_service(0); };
    auto err = espp::task::run_on_core(install_fn, core_id);
    if (err != ESP_OK) {
      logger_.error("Failed to install ISR service: {}", esp_err_to_name(err));
      return;
    }
    ISR_SERVICE_INSTALLED = true;
  }

  struct HandlerArgs {
    int gpio_num;
    QueueHandle_t event_queue;
  };

  struct EventData {
    int gpio_num;
  };

  static void isr_handler(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    auto *args = static_cast<HandlerArgs *>(arg);
#if CONFIG_GPIO_CTRL_FUNC_IN_IRAM
    // disable the interrupt for the pin
    gpio_intr_disable(static_cast<gpio_num_t>(args->gpio_num));
#endif
    EventData event_data = {args->gpio_num};
    xQueueSendFromISR(args->event_queue, &event_data, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
  }

  bool is_active_level(int gpio_num, ActiveLevel active_level) const {
    auto level = gpio_get_level(static_cast<gpio_num_t>(gpio_num));
    return level == static_cast<int>(active_level);
  }

  bool task_callback(std::mutex &, std::condition_variable &, bool &) {
    EventData event_data;
    // record the min number of free spaces in the queue
    size_t free_spaces = uxQueueSpacesAvailable(queue_);
    if (free_spaces < min_queue_size_) {
      min_queue_size_ = free_spaces;
    }
    if (xQueueReceive(queue_, &event_data, portMAX_DELAY)) {
      if (event_data.gpio_num == -1) {
        // we received a stop event, so return true to stop the task
        return true;
      }
      logger_.info("Received interrupt for GPIO {}", event_data.gpio_num);
      std::lock_guard<std::recursive_mutex> lock(interrupt_mutex_);
      // use std::find_if to find the interrupt with the matching gpio_num
      auto predicate = [event_data](const PinConfig &interrupt) {
        return interrupt.gpio_num == event_data.gpio_num;
      };
      auto interrupt = std::find_if(interrupts_.begin(), interrupts_.end(), predicate);
      if (interrupt == interrupts_.end()) {
        logger_.error("No interrupt found for GPIO {}", event_data.gpio_num);
        return false;
      }
#if CONFIG_GPIO_CTRL_FUNC_IN_IRAM
      if (interrupt->auto_reenable) {
        logger_.debug("Auto-reenabling interrupt for GPIO {}", event_data.gpio_num);
        gpio_intr_enable(static_cast<gpio_num_t>(event_data.gpio_num));
      }
#endif
      if (!interrupt->callback) {
        logger_.error("No callback registered for GPIO {}", event_data.gpio_num);
        return false;
      }
      logger_.debug("Calling interrupt callback for GPIO {}", event_data.gpio_num);
      bool active = is_active_level(event_data.gpio_num, interrupt->active_level);
      logger_.debug("GPIO {} is {}", event_data.gpio_num, active ? "active" : "inactive");
      Event event = {static_cast<uint8_t>(event_data.gpio_num), active};
      interrupt->callback(event);
    }
    // we don't want to stop the task, so return false
    return false;
  }

  void configure_filter(const PinConfig &interrupt) {
    if (interrupt.filter_type == FilterType::PIN_GLITCH_FILTER) {
#if CONFIG_SOC_GPIO_SUPPORT_PIN_GLITCH_FILTER
      logger_.info("Enabling pin glitch filter for GPIO {}", interrupt.gpio_num);
      gpio_glitch_filter_handle_t handle;
      gpio_pin_glitch_filter_config_t filter_config;
      memset(&filter_config, 0, sizeof(filter_config));
      filter_config.gpio_num = static_cast<gpio_num_t>(interrupt.gpio_num);
      auto err = gpio_new_pin_glitch_filter(&filter_config, &handle);
      if (err != ESP_OK) {
        logger_.error("Failed to create pin glitch filter for GPIO {}: {}", interrupt.gpio_num,
                      esp_err_to_name(err));
        return;
      }
      // save the handle
      glitch_filter_handles_.push_back(handle);
      // and enable the glitch filter
      gpio_glitch_filter_enable(handle);
#else
      logger_.warn("Glitch filter not supported on this chip");
#endif
    } else if (interrupt.filter_type == FilterType::FLEX_GLITCH_FILTER) {
#if CONFIG_SOC_GPIO_FLEX_GLITCH_FILTER_NUM > 0
      logger_.info("Enabling flex glitch filter for GPIO {}", interrupt.gpio_num);
      gpio_glitch_filter_handle_t handle;
      gpio_flex_glitch_filter_config_t filter_config;
      memset(&filter_config, 0, sizeof(filter_config));
      filter_config.gpio_num = static_cast<gpio_num_t>(interrupt.gpio_num);
      filter_config.window_width_ns = interrupt.filter_config.window_width_ns;
      filter_config.window_thres_ns = interrupt.filter_config.window_threshold_ns;
      auto err = gpio_new_flex_glitch_filter(&filter_config, &handle);
      if (err != ESP_OK) {
        logger_.error("Failed to create flex glitch filter for GPIO {}: {}", interrupt.gpio_num,
                      esp_err_to_name(err));
        return;
      }
      // save the handle
      glitch_filter_handles_.push_back(handle);
      // and enable the glitch filter
      gpio_glitch_filter_enable(handle);
#else
      logger_.warn("Flex glitch filter not supported on this chip");
#endif
    }
  }

  void configure_interrupt(const PinConfig &interrupt) {
    logger_.info("Configuring interrupt for GPIO {}", interrupt.gpio_num);
    logger_.debug("Config: {}", interrupt);
    if (interrupt.callback == nullptr) {
      logger_.error("No callback provided for GPIO {}, not registering interrupt",
                    interrupt.gpio_num);
      return;
    }
    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.pin_bit_mask = 1ULL << interrupt.gpio_num;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = static_cast<gpio_int_type_t>(interrupt.interrupt_type);
    io_conf.pull_up_en = interrupt.pullup_enabled ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en =
        interrupt.pulldown_enabled ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    {
      std::lock_guard<std::recursive_mutex> lock(interrupt_mutex_);
      // add the isr handler
      HandlerArgs *handler_arg = new HandlerArgs{interrupt.gpio_num, queue_};
      handler_args_.push_back(handler_arg);
      gpio_num_t gpio_num = static_cast<gpio_num_t>(interrupt.gpio_num);
      gpio_isr_handler_add(gpio_num, isr_handler, static_cast<void *>(handler_arg));
      // configure the filter if needed
      configure_filter(interrupt);
    }
    gpio_config(&io_conf);
  }

  static bool ISR_SERVICE_INSTALLED;

  QueueHandle_t queue_{nullptr};
  std::atomic<size_t> min_queue_size_{0};
  std::recursive_mutex interrupt_mutex_;
  std::vector<PinConfig> interrupts_;
  std::vector<HandlerArgs *> handler_args_;
  std::vector<gpio_glitch_filter_handle_t> glitch_filter_handles_; // TODO: remove these when the
                                                                   // interrupts are removed
  std::unique_ptr<Task> task_;
};
} // namespace espp

// for printing the interrupt event using libfmt
template <> struct fmt::formatter<espp::Interrupt::Event> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::Interrupt::Event &t, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "Event{{gpio_num={}, active={}}}", t.gpio_num, t.active);
  }
};

// for printing the interrupt type using libfmt
template <> struct fmt::formatter<espp::Interrupt::Type> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext> auto format(espp::Interrupt::Type t, FormatContext &ctx) const {
    switch (t) {
    case espp::Interrupt::Type::ANY_EDGE:
      return fmt::format_to(ctx.out(), "ANY_EDGE");
    case espp::Interrupt::Type::RISING_EDGE:
      return fmt::format_to(ctx.out(), "RISING_EDGE");
    case espp::Interrupt::Type::FALLING_EDGE:
      return fmt::format_to(ctx.out(), "FALLING_EDGE");
    case espp::Interrupt::Type::LOW_LEVEL:
      return fmt::format_to(ctx.out(), "LOW_LEVEL");
    case espp::Interrupt::Type::HIGH_LEVEL:
      return fmt::format_to(ctx.out(), "HIGH_LEVEL");
    }
    return fmt::format_to(ctx.out(), "UNKNOWN");
  }
};

// for printing the active level using libfmt
template <> struct fmt::formatter<espp::Interrupt::ActiveLevel> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::Interrupt::ActiveLevel t, FormatContext &ctx) const {
    switch (t) {
    case espp::Interrupt::ActiveLevel::LOW:
      return fmt::format_to(ctx.out(), "LOW");
    case espp::Interrupt::ActiveLevel::HIGH:
      return fmt::format_to(ctx.out(), "HIGH");
    }
    return fmt::format_to(ctx.out(), "UNKNOWN");
  }
};

// for printing the FilterType using libfmt
template <> struct fmt::formatter<espp::Interrupt::FilterType> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::Interrupt::FilterType t, FormatContext &ctx) const {
    switch (t) {
    case espp::Interrupt::FilterType::NONE:
      return fmt::format_to(ctx.out(), "NONE");
    case espp::Interrupt::FilterType::PIN_GLITCH_FILTER:
      return fmt::format_to(ctx.out(), "PIN_GLITCH_FILTER");
    case espp::Interrupt::FilterType::FLEX_GLITCH_FILTER:
      return fmt::format_to(ctx.out(), "FLEX_GLITCH_FILTER");
    }
    return fmt::format_to(ctx.out(), "UNKNOWN");
  }
};

// for printing the FilterConfig using libfmt
template <> struct fmt::formatter<espp::Interrupt::FilterConfig> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::Interrupt::FilterConfig &t, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "FilterConfig{{window_width_ns={}, window_threshold_ns={}}}",
                          t.window_width_ns, t.window_threshold_ns);
  }
};

// for printing the PinConfig using libfmt
template <> struct fmt::formatter<espp::Interrupt::PinConfig> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::Interrupt::PinConfig &t, FormatContext &ctx) const {
    return fmt::format_to(
        ctx.out(),
        "PinConfig{{gpio_num={}, active_level={}, interrupt_type={}, pullup_enabled={}, "
        "pulldown_enabled={}, filter_type={}, filter_config={}}}",
        t.gpio_num, t.active_level, t.interrupt_type, t.pullup_enabled, t.pulldown_enabled,
        t.filter_type, t.filter_config);
  }
};
