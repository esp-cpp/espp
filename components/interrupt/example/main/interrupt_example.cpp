#include <chrono>
#include <vector>

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

  // make an interrupt for a single gpio
  {
    espp::Interrupt interrupt({
        .interrupts =
            {
                {
                    .gpio_num = GPIO_NUM_0,
                    .callback = callback,
                    .active_level = espp::Interrupt::ActiveLevel::LOW,
                    .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
                    .pullup_enabled = true,
                    .pulldown_enabled = false,
                    .enable_pin_glitch_filter = true,
                },
            },
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
    espp::Interrupt interrupt12({
        .interrupts =
            {
                {
                    .gpio_num = GPIO_NUM_0,
                    .callback = callback,
                    .active_level = espp::Interrupt::ActiveLevel::LOW,
                    .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
                    .pullup_enabled = true,
                    .pulldown_enabled = false,
                    .enable_pin_glitch_filter = true,
                },
            },
        .task_config =
            {
                .name = "Interrupt task",
                .stack_size_bytes = 6192,
                .priority = 5,
            },
        .log_level = espp::Logger::Verbosity::DEBUG,
    });

    espp::Interrupt interrupt0({
        .interrupts =
            {
                {
                    .gpio_num = GPIO_NUM_12,
                    .callback = callback,
                    .active_level = espp::Interrupt::ActiveLevel::LOW,
                    .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
                    .pullup_enabled = true,
                    .pulldown_enabled = false,
                    .enable_pin_glitch_filter = true,
                },
            },
        .task_config =
            {
                .name = "Interrupt 0 task",
                .stack_size_bytes = 6192,
                .priority = 5,
            },
        .log_level = espp::Logger::Verbosity::DEBUG,
    });

    std::this_thread::sleep_for(5s);
  }

  // make a single interrupt for multiple GPIOs
  // make an interrupt for a single gpio
  {
    // Register for interrupts on a few pins (GPIO_NUM_0, GPIO_NUM_12)
    espp::Interrupt interrupt({
        .interrupts =
            {
                {
                    .gpio_num = GPIO_NUM_0,
                    .callback = callback,
                    .active_level = espp::Interrupt::ActiveLevel::LOW,
                    .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
                    .pullup_enabled = true,
                    .pulldown_enabled = false,
                    .enable_pin_glitch_filter = true,
                },
            },
        .task_config =
            {
                .name = "Interrupt task",
                .stack_size_bytes = 6192,
                .priority = 5,
            },
        .log_level = espp::Logger::Verbosity::DEBUG,
    });

    // use the add_interrupt method to add another interrupt
    interrupt.add_interrupt({
        .gpio_num = GPIO_NUM_12,
        .callback = callback,
        .active_level = espp::Interrupt::ActiveLevel::LOW,
        .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
        .pullup_enabled = true,
        .pulldown_enabled = false,
        .enable_pin_glitch_filter = true,
    });

    std::this_thread::sleep_for(5s);
  }

  //! [interrupt example]
}
