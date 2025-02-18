#include <chrono>
#include <vector>

#include "qtpy.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  // create a logger
  espp::Logger logger({.tag = "Qtpy example", .level = espp::Logger::Verbosity::INFO});
  {
    logger.info("starting example");

    //! [qtpy ex1]
    auto& qtpy = espp::QtPy::get();

    // Initialize the Button
    logger.info("Initializing the button");
    auto on_button_pressed = [&](const auto &event) {
      if (event.active) {
        logger.info("Button pressed!");
      } else {
        logger.info("Button released!");
      }
    };
    qtpy.initialize_button(on_button_pressed);

    // Initialize and test the LED
    logger.info("Initializing the LED");
    qtpy.initialize_led();

    logger.info("Turning LED off for 1 second");
    qtpy.led(espp::Rgb(0.0f, 0.0f, 0.0f));
    std::this_thread::sleep_for(1s);

    logger.info("Setting LED to red for 1 second");
    qtpy.led(espp::Rgb(1.0f, 0.0f, 0.0f));
    std::this_thread::sleep_for(1s);

    logger.info("Setting LED to green for 1 second");
    qtpy.led(espp::Rgb(0.0f, 1.0f, 0.0f));
    std::this_thread::sleep_for(1s);

    logger.info("Setting LED to blue for 1 second");
    qtpy.led(espp::Rgb(0.0f, 0.0f, 1.0f));
    std::this_thread::sleep_for(1s);

    // Use a task to rotate the LED through the rainbow using HSV
    auto task_fn = [&qtpy](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      float t = std::chrono::duration<float>(now - start).count();
      // rotate through rainbow colors in hsv based on time, hue is 0-360
      float hue = (cos(t) * 0.5f + 0.5f) * 360.0f;
      espp::Hsv hsv(hue, 1.0f, 1.0f);
      // full brightness (1.0, default) is _really_ bright, so tone it down
      qtpy.led(hsv);
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

    // Now initialize and test the QWIIC I2C bus
    logger.info("Initializing the QWIIC I2C bus");
    qtpy.initialize_qwiic_i2c(); // NOTE: using default i2c config here (400khz)
    auto i2c = qtpy.qwiic_i2c();

    // probe the bus for all addresses and store the ones that were found /
    // responded.
    logger.info("Probing the I2C bus for devices, this may take a while...");
    std::vector<uint8_t> found_addresses;
    for (uint8_t address = 0; address < 128; address++) {
      if (i2c->probe_device(address)) {
        found_addresses.push_back(address);
      }
    }
    // print out the addresses that were found
    if (found_addresses.empty()) {
      logger.info("No devices found on the I2C bus");
    } else {
      logger.info("Found devices at addresses: {::#02x}", found_addresses);
    }

    //! [qtpy ex1]

    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  logger.info("example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
