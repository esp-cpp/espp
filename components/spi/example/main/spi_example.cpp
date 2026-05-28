#include <array>
#include <cstdio>

#include "spi.hpp"

extern "C" void app_main(void) {
  //! [spi example]
  constexpr spi_host_device_t spi_host = SPI2_HOST;
  constexpr auto sclk_gpio = GPIO_NUM_36;
  constexpr auto mosi_gpio = GPIO_NUM_38;
  constexpr auto miso_gpio = GPIO_NUM_35;
  constexpr auto cs_gpio = GPIO_NUM_37;
  constexpr uint8_t read_address = 0x80;

  espp::Spi spi({
      .host = spi_host,
      .sclk_io_num = sclk_gpio,
      .mosi_io_num = mosi_gpio,
      .miso_io_num = miso_gpio,
      .max_transfer_sz = 16,
  });

  std::error_code ec;
  auto device = spi.add_device(
      {
          .address_bits = 8,
          .mode = 0,
          .clock_speed_hz = 1 * 1000 * 1000,
          .cs_io_num = cs_gpio,
          .queue_size = 1,
      },
      ec);
  if (ec || !device) {
    std::printf("Failed to create SPI device: %s\n", ec.message().c_str());
    return;
  }

  std::array<uint8_t, 2> rx_data{};
  if (!device->read(rx_data, {.address = read_address}, ec)) {
    std::printf("SPI read failed: %s\n", ec.message().c_str());
    return;
  }

  std::printf("Read bytes: 0x%02x 0x%02x\n", rx_data[0], rx_data[1]);
  //! [spi example]
}
