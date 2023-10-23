#include <chrono>
#include <vector>

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "logger.hpp"
#include "task.hpp"
#include "tla2528.hpp"

using namespace std::chrono_literals;

// This example is designed to be run on a QtPy ESP32S3, with the BP-ADS7128
// boosterpack plugged in. We connect the I2C pins on the QtPy Qwiic header
// (3.3V, GPIO41 SDA and GPIO40 SCL, and GND) to J1 of the BP-ADS7128: 3.3V (pin
// 1), SCL (pin 9), SDA (pin 10), and GND (pin 22). We also connect the 5V pin
// on the QtPy header to the 5V pin on the BP-ADS7128 (pin 21 of J1). We then
// run this example, and we should see the raw ADC values printed to the
// console. Finally, we connect the ALERT pin of the BP-ADS7128 (pin 13 of J3)
// to GPIO 18 on the QtPy (A0). We should see the ALERT pin pulled low when the
// joystick is pressed.
//
// Table of connections between QtPy and BP-ADS7128:
// QtPy               | BP-ADS7128
// ---------------------------------------
// 3.3V               | 3.3V (J1 pin 1)
// GND                | GND (J1 pin 22)
// Qwiic SDA (GPIO41) | SDA (J1 pin 10)
// Qwiic SCL (GPIO40) | SCL (J1 pin 9)
// 5V                 | 5V (J1 pin 21)
// A0 (GPIO18)        | ALERT (J3 pin 13)
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
// The ALERT pin of the BP-ADS7128 is exposed on J3 pin 13. We can connect this
// pin to a GPIO on the QtPy, and use it to trigger an interrupt when the ALERT
// pin is pulled low. This is useful if we want to use the ALERT pin to trigger
// an interrupt when the ADC value exceeds a certain threshold. In this example,
// we connect the ALERT pin to GPIO 13 on the QtPy.
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
static constexpr auto ALERT_PIN = (GPIO_NUM_18);
static constexpr auto I2C_NUM = (I2C_NUM_1);
static constexpr auto I2C_SCL_IO = (GPIO_NUM_40);
static constexpr auto I2C_SDA_IO = (GPIO_NUM_41);
static constexpr auto I2C_FREQ_HZ = (400 * 1000);
static constexpr auto I2C_TIMEOUT_MS = (10);

static QueueHandle_t gpio_evt_queue;

static void gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "tla2528 example", .level = espp::Logger::Verbosity::INFO});
  // This example shows using the i2c adc (tla2528)
  {
    logger.info("Starting example!");

    //! [tla2528 example]
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
    auto tla_write = [](uint8_t dev_addr, uint8_t *data, size_t data_len) {
      auto err = i2c_master_write_to_device(I2C_NUM, dev_addr, data, data_len,
                                            I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
      if (err != ESP_OK) {
        logger.error("I2C WRITE ERROR: to {:#04x} '{}'", dev_addr, esp_err_to_name(err));
      }
    };
    auto tla_read = [](uint8_t dev_addr, uint8_t *data, size_t data_len) {
      auto err = i2c_master_read_from_device(I2C_NUM, dev_addr, data, data_len,
                                             I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
      if (err != ESP_OK) {
        logger.error("I2C READ ERROR: to {:#04x} '{}'", dev_addr, esp_err_to_name(err));
      }
    };

    // make the actual tla class
    espp::Tla2528 tla(espp::Tla2528::Config{
        .device_address = espp::Tla2528::DEFAULT_ADDRESS,
        .mode = espp::Tla2528::Mode::AUTO_SEQ,
        .analog_inputs = {espp::Tla2528::Channel::CH1, espp::Tla2528::Channel::CH3},
        .digital_inputs = {espp::Tla2528::Channel::CH5},
        .digital_outputs =
            {espp::Tla2528::Channel::CH7}, // On the BP-ADS7128, CH7 is the LED (active low)
        // unordered map of channel to digital output value
        .digital_output_values = {{espp::Tla2528::Channel::CH7, 1}}, // start the LED off
        // enable oversampling / averaging
        .oversampling_ratio = espp::Tla2528::OversamplingRatio::OSR_32,
        .write = tla_write,
        .read = tla_read,
        .log_level = espp::Logger::Verbosity::WARN,
    });

    // set the digital output drive mode to open-drain
    tla.set_digital_output_mode(espp::Tla2528::Channel::CH7, espp::Tla2528::OutputMode::OPEN_DRAIN);

    // make the task which will get the raw data from the I2C ADC
    fmt::print("%time (s), x (mV), y (mV), select pressed\n");
    auto tla_read_task_fn = [&tla](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration<float>(now - start).count();

      // get the analog input data
      auto all_mv = tla.get_all_mv();
      auto x_mv = all_mv[0]; // the first channel is channel 1 (X axis)
      auto y_mv = all_mv[1]; // the second channel is channel 3 (Y axis)

      // alternatively we could get the analog data in a map
      auto mapped_mv = tla.get_all_mv_map();
      x_mv = mapped_mv[espp::Tla2528::Channel::CH1];
      y_mv = mapped_mv[espp::Tla2528::Channel::CH3];

      // NOTE: we could get all digital inputs as a bitmask using
      // get_digital_input_values(), but we'll just get the one we want.
      // If we wanted to get all of them, we could do:
      // auto input_values = tla.get_digital_input_values();
      auto select_value =
          tla.get_digital_input_value(espp::Tla2528::Channel::CH5); // the button is on channel 5
      auto select_pressed = select_value == 0;                      // joystick button is active low

      // use fmt to print so it doesn't have the prefix and can be used more
      // easily as CSV (for plotting using uart_serial_plotter)
      fmt::print("{:.3f}, {:.3f}, {:.3f}, {}\n", elapsed, x_mv, y_mv, select_pressed ? 1 : 0);
      if (select_pressed) {
        tla.set_digital_output_value(espp::Tla2528::Channel::CH7, 0); // turn on the LED
      } else {
        tla.set_digital_output_value(espp::Tla2528::Channel::CH7, 1); // turn off the LED
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

    auto tla_task = espp::Task::make_unique({.name = "TLA",
                                             .callback = tla_read_task_fn,
                                             .stack_size_bytes{8 * 1024},
                                             .log_level = espp::Logger::Verbosity::INFO});
    tla_task->start();
    //! [tla2528 example]

    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }
  // now clean up the i2c driver (by now the task will have stopped, because we
  // left its scope.
  i2c_driver_delete(I2C_NUM);

  logger.info("TLA2528 example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
