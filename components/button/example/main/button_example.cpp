#include <chrono>
#include <vector>

#include "button.hpp"
#include "event_manager.hpp"
#include "serialization.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Button Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [simple button example]
  // create the button
  espp::Button simple_button({
      .name = "Button 12",
      .gpio_num = GPIO_NUM_12,
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .pullup_enabled = false,
      .pulldown_enabled = false,
      .log_level = espp::Logger::Verbosity::WARN,
  });

  for (int i = 0; i < 10; i++) {
    logger.info("Button 12 state: {}", simple_button.is_pressed());
    std::this_thread::sleep_for(1s);
  }
  //! [simple button example]

  //! [button example]
  static auto start = std::chrono::high_resolution_clock::now();
  std::string button_topic = "button/state";

  // create a button
  espp::Button button({
      .name = "Button 12",
      .interrupt_config =
          {
              .gpio_num = GPIO_NUM_12,
              .callback =
                  [&](const espp::Interrupt::Event &event) {
                    auto now = std::chrono::high_resolution_clock::now();
                    auto elapsed = std::chrono::duration<float>(now - start).count();
                    logger.info("[Callback][{:.3f}] Button {} state changed to: {}", elapsed,
                                event.gpio_num, event.active);
                    // serialize the event
                    std::error_code ec;
                    std::vector<uint8_t> buffer;
                    auto bytes_written = espp::serialize(event, buffer);
                    if (bytes_written > 0) {
                      // publish the event
                      espp::EventManager::get().publish(button_topic, buffer);
                    } else {
                      logger.error("Failed to serialize button state");
                    }
                  },
              .active_level = espp::Interrupt::ActiveLevel::LOW,
              .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
              .pullup_enabled = false,
              .pulldown_enabled = false,
          },
      .task_config =
          {
              .name = "Button 12 task",
              .stack_size_bytes = 8192,
              .priority = 5,
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });

  logger.info("Initial button state: {}", button.is_pressed());

  // register subscriber on the button topic
  auto &em = espp::EventManager::get();
  auto did_sub =
      em.add_subscriber(button_topic, "example subscriber", [&](const std::vector<uint8_t> &data) {
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<float>(now - start).count();
        // deserialize the data
        std::error_code ec;
        auto state = espp::deserialize<espp::Interrupt::Event>(data, ec);
        if (!ec) {
          logger.info("[Subscriber][{:.3f}]: button {} state changed to: {}", elapsed,
                      state.gpio_num, state.active);
        } else {
          logger.error("Failed to deserialize button state: {}", ec.message());
        }
      });
  logger.info("Subscribed: {}", did_sub);

  // create another button
  espp::Button button_2({
      .name = "Button 0",
      .interrupt_config =
          {
              .gpio_num = GPIO_NUM_0,
              .callback =
                  [&](const espp::Interrupt::Event &event) {
                    logger.info("Button {} state changed to: {}", event.gpio_num, event.active);
                  },
              .active_level = espp::Interrupt::ActiveLevel::LOW,
              .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
              .pullup_enabled = false,
              .pulldown_enabled = false,
              .filter_type = espp::Interrupt::FilterType::PIN_GLITCH_FILTER,
          },
      .task_config =
          {
              .name = "Button 0 task",
              .stack_size_bytes = 8192,
              .priority = 5,
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });

  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [button example]
}
