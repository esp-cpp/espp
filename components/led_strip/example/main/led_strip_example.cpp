#include <chrono>
#include <vector>

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "led_strip.hpp"
#include "logger.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

static constexpr auto BUS_NUM = (SPI2_HOST);
static constexpr auto CLOCK_IO = (GPIO_NUM_12); // TinyPICO: APA102 CLK
static constexpr auto DATA_IO = (GPIO_NUM_2);   // TinyPICO: APA102 DATA
static constexpr auto POWER_IO = (GPIO_NUM_13); // TinyPICO: APA102 PWR

extern "C" void app_main(void) {
  esp_err_t err;
  // create a logger
  espp::Logger logger({.tag = "Led Strip example", .level = espp::Logger::Verbosity::INFO});
  {
    // configure the power pin
    gpio_config_t power_pin_config = {
        .pin_bit_mask = (1ULL << POWER_IO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&power_pin_config);
    // turn on the power (active low on TinyPICO to save power)
    gpio_set_level(POWER_IO, 0);

    // configure the spi bus
    spi_bus_config_t bus_config;
    memset(&bus_config, 0, sizeof(bus_config));
    bus_config.miso_io_num = -1;
    bus_config.mosi_io_num = DATA_IO;
    bus_config.sclk_io_num = CLOCK_IO;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 100; // NOTE: should make this num_leds * 4
    // initialize the bus
    err = spi_bus_initialize(BUS_NUM, &bus_config, 1);
    if (err != ESP_OK) {
      logger.error("Could not initialize SPI bus: {} - {}", err, esp_err_to_name(err));
    }
    // add the leds to the bus
    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.mode = 0; // SPI mode 0
    devcfg.address_bits = 0;
    devcfg.command_bits = 0;
    devcfg.dummy_bits = 0;
    devcfg.clock_speed_hz = 1 * 1000 * 1000; // Clock out at 1 MHz
    devcfg.input_delay_ns = 0;
    devcfg.spics_io_num = -1;
    devcfg.queue_size = 1;
    // devcfg.flags = SPI_DEVICE_NO_DUMMY;
    spi_device_handle_t spi;
    err = spi_bus_add_device(BUS_NUM, &devcfg, &spi);
    if (err != ESP_OK) {
      logger.error("Could not add SPI device: {} - {}", err, esp_err_to_name(err));
    }

    //! [led strip ex1]
    // create the write function we'll use
    auto apa102_write = [&spi](const uint8_t *data, size_t len) {
      if (len == 0) {
        return;
      }
      static spi_transaction_t t;
      memset(&t, 0, sizeof(t));
      t.length = len * 8;
      t.tx_buffer = data;
      spi_device_polling_transmit(spi, &t);
    };

    // now create the LedStrip object
    espp::LedStrip led_strip(espp::LedStrip::Config{
        .num_leds = 1,
        .write = apa102_write,
        .send_brightness = true,
        .byte_order = espp::LedStrip::ByteOrder::BGR,
        .start_frame = espp::LedStrip::APA102_START_FRAME,
        .log_level = espp::Logger::Verbosity::INFO,
    });

    // Set first pixel using RGB
    led_strip.set_pixel(0, espp::Rgb(0, 255, 255));
    // And show it
    led_strip.show();

    // Use a task to rotate the LED through the rainbow using HSV
    auto task_fn = [&led_strip](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      float t = std::chrono::duration<float>(now - start).count();
      // rotate through rainbow colors in hsv based on time, hue is 0-360
      float hue = (cos(t) * 0.5f + 0.5f) * 360.0f;
      espp::Hsv hsv(hue, 1.0f, 1.0f);
      fmt::print("hsv: {}\n", hsv);
      // full brightness (1.0, default) is _really_ bright, so tone it down
      led_strip.set_pixel(0, hsv, 0.05f);
      led_strip.show();
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 50ms);
      }
      // don't want to stop the task
      return false;
    };

    auto task = espp::Task({.name = "LedStrip Task",
                            .callback = task_fn,
                            .stack_size_bytes = 5 * 1024,
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [led strip ex1]

    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  logger.info("example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
