#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "as5600.hpp"
#include "butterworth_filter.hpp"
#include "i2c.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    std::atomic<bool> quit_test = false;
    fmt::print("Starting as5600 example, rotate to -720 degrees to quit!\n");
    //! [as5600 example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_1,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
    });

    // make the velocity filter
    static constexpr float filter_cutoff_hz = 4.0f;
    static constexpr float encoder_update_period = 0.01f; // seconds
    espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter(
        {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * encoder_update_period});
    // NOTE: we could just as easily disable filtering by simply returning the
    // raw value from this function
    auto filter_fn = [&filter](float raw) -> float { return filter.update(raw); };

    // now make the as5600 which decodes the data
    espp::As5600 as5600(
        {.write_then_read =
             std::bind(&espp::I2c::write_read, &i2c, std::placeholders::_1, std::placeholders::_2,
                       std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
         .velocity_filter = filter_fn,
         .update_period = std::chrono::duration<float>(encoder_update_period),
         .log_level = espp::Logger::Verbosity::WARN});

    // and finally, make the task to periodically poll the as5600 and print the
    // state. NOTE: the As5600 runs its own task to maintain state, so we're
    // just polling the current state.
    auto task_fn = [&quit_test, &as5600](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds = std::chrono::duration<float>(now - start).count();
      auto count = as5600.get_count();
      auto radians = as5600.get_radians();
      auto degrees = as5600.get_degrees();
      auto rpm = as5600.get_rpm();
      fmt::print("{:.3f}, {}, {:.3f}, {:.3f}, {:.3f}\n", seconds, count, radians, degrees, rpm);
      quit_test = degrees <= -720.0f;
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 50ms);
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config =
                                {
                                    .name = "As5600 Task",
                                    .stack_size_bytes = 5 * 1024,
                                },
                            .log_level = espp::Logger::Verbosity::WARN});
    fmt::print("%time(s), count, radians, degrees, rpm\n");
    task.start();
    //! [as5600 example]
    while (!quit_test) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("As5600 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
