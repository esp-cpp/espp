#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "driver/gpio.h"

#include "ads7138.hpp"
#include "i2c.hpp"
#include "logger.hpp"
#include "task.hpp"

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
static QueueHandle_t gpio_evt_queue;

static void gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "ads7138 example", .level = espp::Logger::Verbosity::INFO});
  // This example shows using the i2c adc (ads7138)
  {
    logger.info("Starting example!");

    //! [ads7138 example]
    static constexpr gpio_num_t ALERT_PIN = (gpio_num_t)CONFIG_EXAMPLE_ALERT_GPIO;
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_1,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
    });

    // make the actual ads class
    static int select_bit_mask = (1 << 5);
    espp::Ads7138 ads(espp::Ads7138::Config{
        .device_address = espp::Ads7138::DEFAULT_ADDRESS,
        .mode = espp::Ads7138::Mode::AUTONOMOUS,
        .analog_inputs = {espp::Ads7138::Channel::CH1, espp::Ads7138::Channel::CH3},
        .digital_inputs = {espp::Ads7138::Channel::CH5},
        .digital_outputs =
            {espp::Ads7138::Channel::CH7}, // On the BP-ADS7128, CH7 is the LED (active low)
        // unordered map of channel to digital output value
        .digital_output_values = {{espp::Ads7138::Channel::CH7, 1}}, // start the LED off
        // enable oversampling / averaging
        .oversampling_ratio = espp::Ads7138::OversamplingRatio::OSR_32,
        .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                           std::placeholders::_3),
        .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3),
        .log_level = espp::Logger::Verbosity::WARN,
    });

    // create the gpio event queue
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // setup gpio interrupts for mute button
    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    // interrupt on falling edge since the ALERT pin is pulled up
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1 << static_cast<int>(ALERT_PIN));
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE; // it's pulled up on the BP-ADS7128 as well
    gpio_config(&io_conf);

    // install gpio isr service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ALERT_PIN, gpio_isr_handler, (void *)ALERT_PIN);

    // start the gpio task
    auto alert_task = espp::Task::make_unique({
        .callback = [&ads](auto &m, auto &cv) -> bool {
          static uint32_t io_num;
          // block until we get a message from the interrupt handler
          if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // see if it's the mute button
            if (io_num == (int)ALERT_PIN) {
              // we got an interrupt from the ALERT pin, so read the event data
              uint8_t event_flags = 0;
              uint8_t event_high_flags = 0;
              uint8_t event_low_flags = 0;
              std::error_code ec;
              ads.get_event_data(&event_flags, &event_high_flags, &event_low_flags,
                                 ec); // NOTE: this clears the event flags / ALERT
              if (ec) {
                logger.error("error getting event data: {}", ec.message());
                return false;
              }

              // See if there was an alert on the digital input (Sel, channel 5)
              if (event_flags & select_bit_mask) {
                // See if there was actually a low event on the digital input (Sel, channel 5)
                if (event_low_flags & select_bit_mask) {
                  logger.info("ALERT: Select pressed!");
                }
              }
            }
          }
          // don't want to stop the task
          return false;
        },
        .task_config =
            {
                .name = "alert",
                .stack_size_bytes = 4 * 1024,
            },
    });
    alert_task->start();

    std::error_code ec;
    // configure the alert pin
    ads.configure_alert(espp::Ads7138::OutputMode::PUSH_PULL, espp::Ads7138::AlertLogic::ACTIVE_LOW,
                        ec);
    if (ec) {
      logger.error("error configuring alert: {}", ec.message());
    }

    // set the digital output drive mode to open-drain
    ads.set_digital_output_mode(espp::Ads7138::Channel::CH7, espp::Ads7138::OutputMode::OPEN_DRAIN,
                                ec);
    if (ec) {
      logger.error("error setting digital output mode: {}", ec.message());
    }

    // set an alert on the digital input (Sel) so that we can get notified when the button is
    // pressed (goes low)
    ads.set_digital_alert(espp::Ads7138::Channel::CH5, espp::Ads7138::DigitalEvent::LOW, ec);
    if (ec) {
      logger.error("error setting digital alert: {}", ec.message());
    }

    // make the task which will get the raw data from the I2C ADC
    fmt::print("%time (s), x (mV), y (mV), select pressed\n");
    auto ads_read_task_fn = [&ads](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration<float>(now - start).count();

      std::error_code ec;
      // get the analog input data
      auto all_mv = ads.get_all_mv(ec);
      if (ec) {
        logger.error("error getting analog data: {}", ec.message());
        return false;
      }
      auto x_mv = all_mv[0]; // the first channel is channel 1 (X axis)
      auto y_mv = all_mv[1]; // the second channel is channel 3 (Y axis)

      // alternatively we could get the analog data in a map
      auto mapped_mv = ads.get_all_mv_map(ec);
      if (ec) {
        logger.error("error getting analog data: {}", ec.message());
        return false;
      }

      x_mv = mapped_mv[espp::Ads7138::Channel::CH1];
      y_mv = mapped_mv[espp::Ads7138::Channel::CH3];

      // NOTE: we could get all digital inputs as a bitmask using
      // get_digital_input_values(), but we'll just get the one we want.
      // If we wanted to get all of them, we could do:
      // auto input_values = ads.get_digital_input_values();
      auto select_value = ads.get_digital_input_value(espp::Ads7138::Channel::CH5,
                                                      ec); // the button is on channel 5
      if (ec) {
        logger.error("error getting digital input value: {}", ec.message());
        return false;
      }

      auto select_pressed = select_value == 0; // joystick button is active low

      // use fmt to print so it doesn't have the prefix and can be used more
      // easily as CSV (for plotting using uart_serial_plotter)
      fmt::print("{:.3f}, {:.3f}, {:.3f}, {}\n", elapsed, x_mv, y_mv, select_pressed ? 1 : 0);
      if (select_pressed) {
        ads.set_digital_output_value(espp::Ads7138::Channel::CH7, 0, ec); // turn on the LED
      } else {
        ads.set_digital_output_value(espp::Ads7138::Channel::CH7, 1, ec); // turn off the LED
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

    auto ads_task = espp::Task::make_unique({.callback = ads_read_task_fn,
                                             .task_config =
                                                 {
                                                     .name = "ADS",
                                                     .stack_size_bytes{8 * 1024},
                                                 },
                                             .log_level = espp::Logger::Verbosity::INFO});
    ads_task->start();
    //! [ads7138 example]

    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  logger.info("ADS7138 example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
