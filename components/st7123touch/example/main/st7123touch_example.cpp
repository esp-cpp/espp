#include <chrono>
#include <sdkconfig.h>

#include "i2c.hpp"
#include "st7123touch.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    fmt::print("Starting st7123touch example\n");
    //! [st7123touch example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .timeout_ms = 100,
        .clk_speed = 400 * 1000,
    });

    bool has_st7123 = i2c.probe_device(espp::St7123Touch::DEFAULT_ADDRESS);
    fmt::print("ST7123 touch probe: {}\n", has_st7123);
    fmt::print("         address:   {:#02x}\n", espp::St7123Touch::DEFAULT_ADDRESS);

    // Create the ST7123 touch driver
    espp::St7123Touch touch({
        .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                           std::placeholders::_3),
        .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3),
        .log_level = espp::Logger::Verbosity::WARN,
    });

    // Wrap the driver in a concept-erased ITouchDriver pointer.
    // Any type satisfying espp::TouchDriverConcept can be wrapped this way.
    auto driver = espp::make_touch_driver(std::make_shared<espp::St7123Touch>(std::move(touch)));

    // Poll for touch events using the type-erased interface
    auto task_fn = [&driver](std::mutex &m, std::condition_variable &cv) {
      std::error_code ec;
      bool new_data = driver->update(ec);
      if (ec) {
        fmt::print("Could not update state\n");
        return false;
      }
      if (!new_data) {
        return false; // don't stop the task
      }
      uint8_t num_touch_points = 0;
      uint16_t x = 0, y = 0;
      driver->get_touch_point(&num_touch_points, &x, &y);
      fmt::print("num_touch_points: {}, x: {}, y: {}\n", num_touch_points, x, y);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 50ms);
      }
      return false; // don't stop the task
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config = {.name = "St7123Touch Task"},
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [st7123touch example]
    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("St7123Touch example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
