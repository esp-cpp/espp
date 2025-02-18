#include <chrono>
#include <vector>

#include "neopixel.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  // create a logger
  espp::Logger logger({.tag = "Neopixel example", .level = espp::Logger::Verbosity::INFO});
  {
    logger.info("starting example");

    //! [neopixel ex1]
    espp::Neopixel led({
        .data_gpio = 39,  // Neopixel data pin on QtPy ESP32s3
        .power_gpio = 38, // Neopixel power pin on QtPy ESP32s3
    });

    logger.info("Turning LED off for 1 second");
    led.set_color(espp::Rgb(0.0f, 0.0f, 0.0f));
    led.show();
    std::this_thread::sleep_for(1s);

    logger.info("Setting LED to red for 1 second");
    led.set_color(espp::Rgb(1.0f, 0.0f, 0.0f));
    led.show();
    std::this_thread::sleep_for(1s);

    logger.info("Setting LED to green for 1 second");
    led.set_color(espp::Rgb(0.0f, 1.0f, 0.0f));
    led.show();
    std::this_thread::sleep_for(1s);

    logger.info("Setting LED to blue for 1 second");
    led.set_color(espp::Rgb(0.0f, 0.0f, 1.0f));
    led.show();
    std::this_thread::sleep_for(1s);

    // Use a task to rotate the LED through the rainbow using HSV
    auto task_fn = [&led](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      float t = std::chrono::duration<float>(now - start).count();
      // rotate through rainbow colors in hsv based on time, hue is 0-360
      float hue = (cos(t) * 0.5f + 0.5f) * 360.0f;
      espp::Hsv hsv(hue, 1.0f, 1.0f);
      // full brightness (1.0, default) is _really_ bright, so tone it down
      led.set_color(hsv);
      // show the new colors
      led.show();
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 50ms);
      }
      // don't want to stop the task
      return false;
    };

    logger.info("Starting task to rotate LED through rainbow colors");
    auto task = espp::Task({.callback = task_fn,
                            .task_config =
                                {
                                    .name = "Neopixel Task",
                                    .stack_size_bytes = 5 * 1024,
                                },
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [neopixel ex1]

    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  logger.info("example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
