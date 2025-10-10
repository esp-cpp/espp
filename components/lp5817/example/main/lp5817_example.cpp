#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "i2c.hpp"
#include "lp5817.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "LP5817 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");

  //! [lp5817 example]
  espp::I2c i2c({
      .port = I2C_NUM_0,
      .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .clk_speed = CONFIG_EXAMPLE_I2C_CLOCK_SPEED_HZ,
      .log_level = espp::Logger::Verbosity::INFO,
  });

  espp::Lp5817 lp({.device_address = espp::Lp5817::DEFAULT_ADDRESS,
                   .write = std::bind_front(&espp::I2c::write, &i2c),
                   .write_then_read = std::bind_front(&espp::I2c::write_read, &i2c),
                   .log_level = espp::Logger::Verbosity::WARN});
  std::error_code ec;
  if (!lp.initialize(ec)) {
    logger.error("Failed to initialize LP5817: {}", ec.message());
    return;
  }

  // configure the max current per channel
  static constexpr auto max_current = espp::Lp5817::GlobalMaxCurrent::MA_51;
  logger.info("Setting max current to {}", max_current);
  if (!lp.set_max_current(max_current, ec)) {
    logger.error("Failed to set max current: {}", ec.message());
    return;
  }

  // configure the fade time
  static constexpr auto fade_time = espp::Lp5817::FadeTime::TIME_300MS;
  logger.info("Setting fade time to {}", fade_time);
  if (!lp.set_fade_time(fade_time, ec)) {
    logger.error("Failed to set fade time: {}", ec.message());
    return;
  }

  // enable fading and exponential dimming on all channels
  using Ch = espp::Lp5817::Channel;
  for (auto ch : {Ch::OUT0, Ch::OUT1, Ch::OUT2}) {
    // enable output on the channel
    if (!lp.set_output_enable(ch, true, ec)) {
      logger.error("Failed to enable output on channel {}: {}", ch, ec.message());
      return;
    }
    // set the dot current (DC) according to how much current you want to drive
    uint8_t dc_value = 0x50; // about 80/255 of max current
    if (!lp.set_dot_current(ch, dc_value, ec)) {
      logger.error("Failed to set dot-current on channel {}: {}", ch, ec.message());
      return;
    }
    // enable fading on the channel
    if (!lp.set_fade_enable(ch, true, ec)) {
      logger.error("Failed to enable fading on channel {}: {}", ch, ec.message());
      return;
    }
    // enable exponential dimming on the channel
    if (!lp.set_exponential_dimming_enable(ch, true, ec)) {
      logger.error("Failed to enable exponential dimming on channel {}: {}", ch, ec.message());
      return;
    }
  }

  // Latch the settings (update)
  if (!lp.update(ec)) {
    logger.error("Failed to latch settings: {}", ec.message());
    return;
  }

  // Simple breathing RGB demo
  auto task_fn = [&](std::mutex &m, std::condition_variable &cv) {
    static float brightness = 0.0f;
    brightness = (brightness == 0.0f) ? 1.0f : 0.0f;
    // for now we'll just to white and fade it in and out
    float r = brightness;
    float g = brightness;
    float b = brightness;
    if (!lp.set_rgb_pwm(r, g, b, ec)) {
      logger.error("LP5817 set_rgb failed: {}", ec.message().c_str());
      return true;
    }
    {
      std::unique_lock<std::mutex> lk(m);
      // waiting for 300ms since that is our fade time
      cv.wait_for(lk, 300ms);
    }
    return false;
  };

  auto task = espp::Task({.callback = task_fn,
                          .task_config = {.name = "LP5817 Task", .stack_size_bytes = 4 * 1024},
                          .log_level = espp::Logger::Verbosity::WARN});
  task.start();
  //! [lp5817 example]

  while (true) {
    std::this_thread::sleep_for(100ms);
  }
}
