#include <chrono>
#include <vector>

#include "driver/i2c.h"

#include "ads7138.hpp"
#include "logger.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

// This example is designed to be run on a QtPy ESP32S3, with the BP-ADS7128
// boosterpack plugged in. We connect the I2C pins on the QtPy Qwiic header
// (3.3V, GPIO41 SDA and GPIO40 SCL, and GND) to J1 of the BP-ADS7128: 3.3V (pin
// 1), SCL (pin 9), SDA (pin 10), and GND (pin 22). We also connect the 5V pin
// on the QtPy header to the 5V pin on the BP-ADS7128 (pin 21 of J1). We then
// run this example, and we should see the raw ADC values printed to the
// console.
//
// Table of connections between QtPy and BP-ADS7128:
// QtPy  | BP-ADS7128
// -------------------
// 3.3V  | 3.3V (pin 1)
// GND   | GND (pin 22)
// GPIO41| SDA (pin 10)
// GPIO40| SCL (pin 9)
// 5V    | 5V (pin 21)
//
// Connected to the BP-ADS7128, we have a Adafruit Thumb Joystick. The X axis
// of the joystick is connected to channel 1 of the BP-ADS7128, and the Y axis
// of the joystick is connected to channel 3 of the BP-ADS7128. The joystick
// also has a button, which is connected to channel 5 of the BP-ADS7128. We
// should see the raw ADC values of the joystick printed to the console, and
// when we press the button on the joystick, we should see the digital value
// change from 1 to 0.
//
// Table of connections between BP-ADS7128 and Adafruit Thumb Joystick:
// BP-ADS7128      | Joystick
// --------------------------------
// CH1 (J5 pin 2)  | X axis (Xout)
// GND (J5 pin 4)  | GND
// CH3 (J5 pin 6)  | Y axis (Yout)
// CH5 (J5 pin 8)  | Button (Sel)
// 3.3V (J1 pin 1) | VCC
//
// NOTE: The analog inputs on J5 of the BP-ADS7128 are connected via a 10k
// resistor in series to the actual analog input of the ADS7128. This is to
// protect the ADS7128 from overvoltage. Therefore you may want to add a pull-up
// resistor to the digital input (Sel) of the joystick, so that when the button
// is not pressed, the digital input is pulled high. Otherwise, the digital
// input will be floating, and the ADS7128 will read a random value.
//
// On the BP-ADS7128, we also have an LED connected to channel 7. This LED is
// active low, so when we set the digital output to 1, the LED should turn off,
// and when we set the digital output to 0, the LED should turn on. We can
// change the digital output value by pressing the button on the joystick.
static constexpr auto I2C_NUM = (I2C_NUM_1);
static constexpr auto I2C_SCL_IO = (GPIO_NUM_40);
static constexpr auto I2C_SDA_IO = (GPIO_NUM_41);
static constexpr auto I2C_FREQ_HZ = (400 * 1000);
static constexpr auto I2C_TIMEOUT_MS = (10);

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "ads7138 example", .level = espp::Logger::Verbosity::INFO});
  // This example shows using the i2c adc (ads7138)
  {
    logger.info("Starting example!");

    //! [ads7138 example]
    // make the I2C that we'll use to communicate
    i2c_config_t i2c_cfg;
    logger.info("initializing i2c driver...");
    memset(&i2c_cfg, 0, sizeof(i2c_cfg));
    i2c_cfg.sda_io_num = I2C_SDA_IO;
    i2c_cfg.scl_io_num = I2C_SCL_IO;
    i2c_cfg.mode = I2C_MODE_MASTER;
    i2c_cfg.sda_pullup_en = GPIO_PULLUP_DISABLE;
    i2c_cfg.scl_pullup_en = GPIO_PULLUP_DISABLE;
    i2c_cfg.master.clk_speed = I2C_FREQ_HZ;
    auto err = i2c_param_config(I2C_NUM, &i2c_cfg);
    if (err != ESP_OK)
      logger.error("config i2c failed");
    err = i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK)
      logger.error("install i2c driver failed");

    // make some lambda functions we'll use to read/write to the i2c adc
    auto ads_write = [](uint8_t dev_addr, uint8_t *data, size_t data_len) {
      auto err = i2c_master_write_to_device(I2C_NUM, dev_addr, data, data_len,
                                            I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
      if (err != ESP_OK) {
        logger.error("I2C WRITE ERROR: to {:#04x} '{}'", dev_addr, esp_err_to_name(err));
      }
    };
    auto ads_read = [](uint8_t dev_addr, uint8_t *data, size_t data_len) {
      auto err = i2c_master_read_from_device(I2C_NUM, dev_addr, data, data_len,
                                             I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
      if (err != ESP_OK) {
        logger.error("I2C READ ERROR: to {:#04x} '{}'", dev_addr, esp_err_to_name(err));
      }
    };

    // make the actual ads class
    espp::Ads7138 ads(espp::Ads7138::Config{
        .device_address = espp::Ads7138::DEFAULT_ADDRESS,
        .mode = espp::Ads7138::Mode::AUTONOMOUS,
        .analog_inputs = {espp::Ads7138::Channel::CH1, espp::Ads7138::Channel::CH3},
        .digital_inputs = {espp::Ads7138::Channel::CH5},
        .digital_outputs =
            {espp::Ads7138::Channel::CH7}, // On the BP-ADS7128, CH7 is the LED (active low)
        // enable oversampling / averaging
        .oversampling_ratio = espp::Ads7138::OversamplingRatio::OSR_32,
        .write = ads_write,
        .read = ads_read,
        .log_level = espp::Logger::Verbosity::WARN,
    });

    // set the digital output drive mode to open-drain
    ads.set_digital_output_mode(espp::Ads7138::Channel::CH7, espp::Ads7138::OutputMode::OPEN_DRAIN);

    // set the digital output to 1 (turn off the LED)
    ads.set_digital_output_value(espp::Ads7138::Channel::CH7, 1);

    // make the task which will get the raw data from the I2C ADC
    fmt::print("%time (s), x (mV), y (mV), select pressed\n");
    auto ads_read_task_fn = [&ads](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration<float>(now - start).count();
      auto all_mv = ads.get_all_mv();
      auto x_mv = all_mv[0]; // the first channel is channel 1 (X axis)
      auto y_mv = all_mv[1]; // the second channel is channel 3 (Y axis)
      auto input_values = ads.get_digital_input_values();
      auto select =
          ads.get_digital_input_value(espp::Ads7138::Channel::CH5); // the button is on channel 5
      auto select_pressed = select == 0;                            // joystick button is active low
      // use fmt to print so it doesn't have the prefix and can be used more
      // easily as CSV also pad the binary value with 0s so it's easier to read
      // (and make sure it's fixed with a chararacter width of 10 - 8 bits + 2
      // for the 0b prefix)
      fmt::print("{:.3f}, {:.3f}, {:.3f}, {}\n", elapsed, x_mv, y_mv, select_pressed ? 1 : 0);
      if (select_pressed) {
        ads.set_digital_output_value(espp::Ads7138::Channel::CH7, 0); // turn on the LED
      } else {
        ads.set_digital_output_value(espp::Ads7138::Channel::CH7, 1); // turn off the LED
      }
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        using namespace std::chrono_literals;
        std::unique_lock<std::mutex> lk(m);
        cv.wait_until(lk, now + 10ms);
      }
      // we don't want to stop, so return false
      return false;
    };

    auto ads_task = espp::Task::make_unique({.name = "ADS",
                                             .callback = ads_read_task_fn,
                                             .stack_size_bytes{8 * 1024},
                                             .log_level = espp::Logger::Verbosity::INFO});
    ads_task->start();
    //! [ads7138 example]

    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }
  // now clean up the i2c driver (by now the task will have stopped, because we
  // left its scope.
  i2c_driver_delete(I2C_NUM);

  logger.info("ADS7138 example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
