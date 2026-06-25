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

    // Create the ST7123 touch driver. It owns a recursive_mutex and atomics, so
    // it is non-copyable and non-movable — construct it directly inside the
    // shared_ptr rather than moving a stack instance into it.
    auto touch = std::make_shared<espp::St7123Touch>(espp::St7123Touch::Config{
        // The ST7123 only holds its register pointer within a single I2C
        // transaction, so register reads must be a repeated-START
        // write-then-read rather than a separate write + read.
        .write_then_read = [&i2c](uint8_t addr, const uint8_t *wdata, size_t wlen, uint8_t *rdata,
                                  size_t rlen) -> bool {
          return i2c.write_read(addr, wdata, wlen, rdata, rlen);
        },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    // Wrap the driver in a concept-erased ITouchDriver pointer.
    // Any type satisfying espp::TouchDriverConcept can be wrapped this way.
    auto driver = espp::make_touch_driver(touch);

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
