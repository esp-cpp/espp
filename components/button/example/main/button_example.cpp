#include <chrono>
#include <vector>

#include "button.hpp"
#include "event_manager.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Button Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [button example]
  static auto start = std::chrono::high_resolution_clock::now();
  std::string button_topic = "button/state";
  std::string button_component_name = "button";

  // create a button on GPIO 2
  espp::Button button({
      .gpio_num = GPIO_NUM_2,
      .topic = button_topic,
      .component_name = button_component_name,
      .active_level = espp::Button::ActiveLevel::HIGH,
      .interrupt_type = espp::Button::InterruptType::ANY_EDGE,
      .pullup_enabled = false,
      .pulldown_enabled = false,
  });

  logger.info("Initial button state: {}", button.is_pressed());

  // register subscriber on the button topic
  auto &em = espp::EventManager::get();
  auto did_sub =
      em.add_subscriber(button_topic, "example subscriber", [&](const std::string &data) {
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<float>(now - start).count();
        // deserialize the data
        std::error_code ec;
        auto state = espp::deserialize<espp::Button::Event>(data, ec);
        if (!ec) {
          logger.info("[{:.3f}]: button {} state changed to: {}", elapsed, state.gpio_num,
                      state.pressed);
        } else {
          logger.error("Failed to deserialize button state: {}", ec.message());
        }
      });
  logger.info("Subscribed: {}", did_sub);

  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [button example]
}
