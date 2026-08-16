#include <chrono>
#include <thread>

#include "gps.hpp"
#include "logger.hpp"

#if CONFIG_EXAMPLE_HARDWARE_CARDPUTER_ADV
#include "i2c.hpp"
#endif

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "GPS Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

#if CONFIG_EXAMPLE_HARDWARE_CARDPUTER_ADV
  // The Cardputer-Adv LoRa+GPS Cap (U214) gates the GPS LNA behind P1 of a
  // PI4IOE5V6408 I2C IO expander; drive it high for better sensitivity.
  // (P0 is the LoRa antenna switch.)
  espp::I2c i2c({.port = I2C_NUM_0,
                 .sda_io_num = GPIO_NUM_8,
                 .scl_io_num = GPIO_NUM_9,
                 .sda_pullup_en = GPIO_PULLUP_ENABLE,
                 .scl_pullup_en = GPIO_PULLUP_ENABLE});
  {
    std::error_code ec;
    auto expander = i2c.add_device<uint8_t>({.device_address = 0x43}, ec);
    if (expander) {
      const uint8_t init_regs[][2] = {
          {0x03, 0x03}, // direction: P0, P1 outputs
          {0x05, 0x03}, // output state: P0, P1 high
          {0x07, 0x00}, // output high-impedance: disabled
      };
      for (const auto &reg : init_regs) {
        expander->write(reg, 2, ec);
      }
    } else {
      logger.warn("Could not configure IO expander: {}", ec.message());
    }
  }
#endif

  //! [gps example]
  espp::Gps gps({.uart_port = UART_NUM_1,
                 .tx_io_num = (gpio_num_t)CONFIG_EXAMPLE_GPS_TX_GPIO,
                 .rx_io_num = (gpio_num_t)CONFIG_EXAMPLE_GPS_RX_GPIO,
                 .baud_rate = CONFIG_EXAMPLE_GPS_BAUD_RATE,
                 .on_fix =
                     [&](const espp::GpsFix &fix) {
                       if (fix.valid) {
                         logger.info("Fix: {:.6f}, {:.6f} alt {:.1f} m, {} sats, hdop {:.1f}, "
                                     "{:02}:{:02}:{:04.1f} UTC",
                                     fix.latitude, fix.longitude, fix.altitude, fix.num_satellites,
                                     fix.hdop, fix.hour, fix.minute, fix.second);
                       } else {
                         logger.info("Waiting for fix ({} sats in use)...", fix.num_satellites);
                       }
                     },
                 .log_level = espp::Logger::Verbosity::INFO});
  //! [gps example]

  while (true) {
    std::this_thread::sleep_for(10s);
  }
}
