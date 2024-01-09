#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include <driver/gpio.h>

#include "i2c.hpp"
#include "t_keyboard.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    fmt::print("Starting tkeyboard example\n");
    //! [tkeyboard example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
    });
    // now make the tkeyboard which decodes the data
    espp::TKeyboard tkeyboard(
        {.write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                            std::placeholders::_3),
         .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1, std::placeholders::_2,
                           std::placeholders::_3),
         .key_cb = [](uint8_t key) { fmt::print("'{}' Pressed!\n", (char)key); },
         .auto_start = false, // can't auto start since we need to provide power
         .log_level = espp::Logger::Verbosity::WARN});

    // on the LilyGo T-Deck, the peripheral power control pin must be set high
    // to enable peripheral power
    auto power_ctrl = GPIO_NUM_10;
    gpio_set_direction(power_ctrl, GPIO_MODE_OUTPUT);
    gpio_set_level(power_ctrl, 1);

    do {
      fmt::print("Waiting for tkeyboard to boot up...\n");
      std::this_thread::sleep_for(250ms);
    } while (!i2c.probe_device(espp::TKeyboard::DEFAULT_ADDRESS));

    fmt::print("Tkeyboard ready!\n");
    tkeyboard.start();
    //! [tkeyboard example]
    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("Tkeyboard example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
