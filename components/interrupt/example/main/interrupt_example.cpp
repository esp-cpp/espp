#include <chrono>
#include <vector>

#include "esp_idf_version.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#include <esp_intr_alloc.h>
#endif // ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)

#include "interrupt.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Interrupt Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [interrupt example]
  static auto start = std::chrono::high_resolution_clock::now();

  auto callback = [&](const espp::Interrupt::Event &event) {
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration<float>(now - start).count();
    logger.info("[Callback][{:.3f}] Interrupt: pin {} changed to active state: {}", elapsed,
                event.gpio_num, event.active);
  };

  espp::Interrupt::PinConfig io0 = {
      .gpio_num = GPIO_NUM_0,
      .callback = callback,
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
      .pullup_enabled = true,
      .pulldown_enabled = false,
      // flexible filter requiring configuration (default is provided as 5us
      // threshold in 10us window), but other configurations can be manually
      // set as below
      .filter_type = espp::Interrupt::FilterType::FLEX_GLITCH_FILTER,
      .filter_config = {.window_width_ns = 10000, .window_threshold_ns = 5000},
  };
  espp::Interrupt::PinConfig io12 = {
      .gpio_num = GPIO_NUM_12,
      .callback = callback,
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
      .pullup_enabled = true,
      .pulldown_enabled = false,
      // pre-configured 2 clock pulse width filter
      .filter_type = espp::Interrupt::FilterType::PIN_GLITCH_FILTER,
  };

  // make an interrupt for a single gpio
  {
    espp::Interrupt interrupt({
        .isr_core_id = 1,
        .interrupts = {io0},
        .task_config =
            {
                .name = "Interrupt task",
                .stack_size_bytes = 6192,
                .priority = 5,
            },
        .log_level = espp::Logger::Verbosity::DEBUG,
    });

    std::this_thread::sleep_for(5s);
  }

  // make multiple interrupts for multiple gpios
  {
    espp::Interrupt interrupt0({
        .interrupts = {io0},
        .task_config =
            {
                .name = "Interrupt task",
                .stack_size_bytes = 6192,
                .priority = 5,
            },
        .log_level = espp::Logger::Verbosity::DEBUG,
    });

    espp::Interrupt interrupt12({
        .interrupts = {io12},
        .task_config =
            {
                .name = "Interrupt 0 task",
                .stack_size_bytes = 6192,
                .priority = 5,
            },
        .log_level = espp::Logger::Verbosity::DEBUG,
    });

    std::this_thread::sleep_for(2s);

    // now lets read the instantaneous state of the interrupt pins
    auto is_0_active = interrupt0.is_active(io0);
    auto is_12_active = interrupt12.is_active(io12);
    logger.info("Instantaneous state of pin 0: {}", is_0_active);
    logger.info("Instantaneous state of pin 12: {}", is_12_active);

    std::this_thread::sleep_for(2s);
  }

  // make a single interrupt for multiple GPIOs
  // make an interrupt for a single gpio
  {
    // Register for interrupts on a few pins (GPIO_NUM_0, GPIO_NUM_12)
    espp::Interrupt interrupt({
        .interrupts = {io0},
        .task_config =
            {
                .name = "Interrupt task",
                .stack_size_bytes = 6192,
                .priority = 5,
            },
        .log_level = espp::Logger::Verbosity::DEBUG,
    });

    // use the add_interrupt method to add another interrupt
    interrupt.add_interrupt(io12);

    std::this_thread::sleep_for(2s);

    // now lets read the instantaneous state of the interrupt pins
    auto active_states = interrupt.get_active_states();
    logger.info("Instantaneous state of pins: {}", active_states);

    std::this_thread::sleep_for(2s);

    // print out the minimum number of spaces in the interrupt queue over the
    // last 2 seconds
    auto min_queue_size = interrupt.get_min_queue_size();
    logger.info("Minimum queue size over last 4 seconds: {}", min_queue_size);
  }

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
  esp_intr_dump(stdout);
#endif // ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)

  //! [interrupt example]
}
