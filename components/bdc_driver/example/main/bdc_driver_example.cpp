#include <cmath>
#include <numbers>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "bdc_driver.hpp"
#include "logger.hpp"

namespace {
static auto logger = espp::Logger({
    .tag = "BdcDriver Example",
    .level = espp::Logger::Verbosity::INFO,
});
} // namespace

extern "C" void app_main() {
  //! [bdc_driver example]
  constexpr gpio_num_t motor_gpio_a = GPIO_NUM_16;
  constexpr gpio_num_t motor_gpio_b = GPIO_NUM_17;

  espp::BdcDriver driver({
      .gpio_a = motor_gpio_a,
      .gpio_b = motor_gpio_b,
      .group_id = 0,
      .log_level = espp::Logger::Verbosity::INFO,
  });

  if (!driver.initialized()) {
    logger.error("Failed to initialize BDC driver");
    return;
  }

  float phase = 0.0f;
  while (true) {
    float command = 0.8f * std::sin(phase);
    driver.set_speed(command);
    auto duty = driver.duty_cycle();
    auto raw = driver.raw_duty();
    logger.info("cmd={:.3f} | pwm_a duty={:.1f}% raw={}/{} | pwm_b duty={:.1f}% raw={}/{}", command,
                duty[0] * 100.0f, raw[0], driver.max_raw_duty(), duty[1] * 100.0f, raw[1],
                driver.max_raw_duty());
    phase = std::fmod(phase + 0.15f, 2.0f * std::numbers::pi_v<float>);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  //! [bdc_driver example]
}
