#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <driver/gpio.h>

#include "interrupt.hpp"
#include "logger.hpp"
#include "spi.hpp"
#include "sx126x.hpp"
#include "task.hpp"

#if CONFIG_EXAMPLE_HARDWARE_CARDPUTER_ADV
#include "i2c.hpp"
#endif

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "SX126x Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  static constexpr gpio_num_t spi_sclk = (gpio_num_t)CONFIG_EXAMPLE_SPI_SCLK_GPIO;
  static constexpr gpio_num_t spi_mosi = (gpio_num_t)CONFIG_EXAMPLE_SPI_MOSI_GPIO;
  static constexpr gpio_num_t spi_miso = (gpio_num_t)CONFIG_EXAMPLE_SPI_MISO_GPIO;
  static constexpr gpio_num_t radio_cs = (gpio_num_t)CONFIG_EXAMPLE_RADIO_CS_GPIO;
  static constexpr gpio_num_t radio_dio1 = (gpio_num_t)CONFIG_EXAMPLE_RADIO_DIO1_GPIO;
  static constexpr gpio_num_t radio_busy = (gpio_num_t)CONFIG_EXAMPLE_RADIO_BUSY_GPIO;
  static constexpr gpio_num_t radio_reset = (gpio_num_t)CONFIG_EXAMPLE_RADIO_RESET_GPIO;

#if CONFIG_EXAMPLE_POWER_ENABLE_GPIO >= 0
  // some boards (e.g. T-Deck) gate power to the radio behind a GPIO
  static constexpr gpio_num_t power_enable = (gpio_num_t)CONFIG_EXAMPLE_POWER_ENABLE_GPIO;
  gpio_set_direction(power_enable, GPIO_MODE_OUTPUT);
  gpio_set_level(power_enable, 1);
  std::this_thread::sleep_for(100ms);
#endif

#if CONFIG_EXAMPLE_HARDWARE_CARDPUTER_ADV
  // The Cardputer-Adv LoRa+GPS Cap (U214) routes the radio's RF switch
  // through P0 of a PI4IOE5V6408 I2C IO expander - it must be driven high to
  // connect the antenna. P1 enables the GPS LNA.
  espp::I2c i2c({.port = I2C_NUM_0,
                 .sda_io_num = GPIO_NUM_8,
                 .scl_io_num = GPIO_NUM_9,
                 .sda_pullup_en = GPIO_PULLUP_ENABLE,
                 .scl_pullup_en = GPIO_PULLUP_ENABLE});
  {
    static constexpr uint8_t expander_address = 0x43;
    std::error_code ec;
    auto expander = i2c.add_device<uint8_t>({.device_address = expander_address}, ec);
    if (!expander) {
      logger.error("Failed to add IO expander device: {}", ec.message());
      return;
    }
    // set P0 (RF switch) and P1 (GPS LNA) as outputs, driven high, not Hi-Z
    const uint8_t init_regs[][2] = {
        {0x03, 0x03}, // direction: P0, P1 outputs
        {0x05, 0x03}, // output state: P0, P1 high
        {0x07, 0x00}, // output high-impedance: disabled
    };
    for (const auto &reg : init_regs) {
      expander->write(reg, 2, ec);
      if (ec) {
        logger.error("Failed to configure IO expander: {}", ec.message());
        return;
      }
    }
  }
#endif

  //! [sx126x example]
  // create the (shared) SPI bus and add the radio as a device on it
  espp::Spi spi({.host = SPI2_HOST,
                 .sclk_io_num = spi_sclk,
                 .mosi_io_num = spi_mosi,
                 .miso_io_num = spi_miso});
  std::error_code ec;
  auto radio_device = spi.add_device(
      {.mode = 0, .clock_speed_hz = 8 * 1000 * 1000, .cs_io_num = radio_cs, .queue_size = 1}, ec);
  if (!radio_device) {
    logger.error("Failed to add radio SPI device: {}", ec.message());
    return;
  }

  // configure the radio's control pins
  gpio_set_direction(radio_busy, GPIO_MODE_INPUT);
  gpio_set_direction(radio_reset, GPIO_MODE_OUTPUT);

  // The SX126x requires chip select to be held asserted across the write and
  // read phases of a command, so implement write_then_read as a single
  // full-duplex transfer of the concatenated length.
  auto write_fn = [&](const uint8_t *data, size_t length) -> bool {
    std::error_code ec;
    return radio_device->write(std::span{data, length}, {}, ec);
  };
  auto write_then_read_fn = [&](const uint8_t *write_data, size_t write_length, uint8_t *read_data,
                                size_t read_length) -> bool {
    // This is a hot path (called for every SX126x command / IRQ read), so use
    // fixed, DMA-aligned stack buffers instead of allocating a vector pair on
    // every call. The largest SX126x transfer is a 255-byte FIFO read plus a
    // few opcode/param bytes, so kMaxTransfer comfortably covers any command.
    static constexpr size_t kMaxTransfer = 264;
    const size_t total = write_length + read_length;
    if (total > kMaxTransfer) {
      return false;
    }
    alignas(4) uint8_t tx[kMaxTransfer] = {0};
    alignas(4) uint8_t rx[kMaxTransfer];
    std::memcpy(tx, write_data, write_length);
    std::error_code ec;
    if (!radio_device->transfer(std::span{tx, total}, std::span{rx, total}, {}, ec)) {
      return false;
    }
    std::memcpy(read_data, rx + write_length, read_length);
    return true;
  };

  // make the radio
  espp::Sx126x radio({
    .variant = espp::Sx126x::Variant::SX1262, .write = write_fn,
    .write_then_read = write_then_read_fn,
    .is_busy = [&]() -> bool { return gpio_get_level(radio_busy); },
    .reset = [&](bool level) { gpio_set_level(radio_reset, level); },
#if CONFIG_EXAMPLE_RADIO_HAS_TCXO
    .tcxo_voltage = 1.8f,
#endif
    .radio_config =
        {
            .frequency_hz = CONFIG_EXAMPLE_RADIO_FREQUENCY_HZ,
            .tx_power_dbm = 22,
            .spreading_factor = espp::Sx126x::SpreadingFactor::SF11,
            .bandwidth = espp::Sx126x::Bandwidth::BW_250_KHZ,
            .coding_rate = espp::Sx126x::CodingRate::CR_4_5,
            .preamble_length = 16,
            .sync_word = 0x2B,
        },
    .on_receive =
        [&](const espp::Sx126x::RxPacket &packet) {
          std::string text(packet.data.begin(), packet.data.end());
          logger.info("Received {} bytes (RSSI {:.1f} dBm, SNR {:.2f} dB): '{}'",
                      packet.data.size(), packet.status.rssi, packet.status.snr, text);
        },
    .on_transmit_done = [&]() { logger.info("Transmit complete"); }, .auto_init = false,
    .log_level = espp::Logger::Verbosity::INFO
  });
  if (!radio.initialize(ec)) {
    logger.error("Failed to initialize radio: {}", ec.message());
    return;
  }

  // service the radio's IRQs (from a task context) whenever DIO1 goes high
  espp::Interrupt interrupts(
      {.interrupts = {{.gpio_num = radio_dio1,
                       .callback =
                           [&](const espp::Interrupt::Event &event) {
                             std::error_code ec;
                             radio.handle_dio1_interrupt(ec);
                           },
                       .active_level = espp::Interrupt::ActiveLevel::HIGH,
                       .interrupt_type = espp::Interrupt::Type::RISING_EDGE}},
       .task_config = {.name = "radio_interrupts", .stack_size_bytes = 6 * 1024}});

  // listen for packets, and send a ping every transmit interval
  if (!radio.start_receive(ec)) {
    logger.error("Failed to start receiving: {}", ec.message());
    return;
  }
  logger.info("Radio listening on {:.3f} MHz", CONFIG_EXAMPLE_RADIO_FREQUENCY_HZ / 1e6f);

  int packet_count = 0;
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(CONFIG_EXAMPLE_TRANSMIT_INTERVAL_MS));
    std::string message = fmt::format("ping {} from sx126x example", packet_count++);
    logger.info("Transmitting: '{}'", message);
    std::span<const uint8_t> payload{reinterpret_cast<const uint8_t *>(message.data()),
                                     message.size()};
    if (!radio.transmit(payload, 5s, ec)) {
      logger.error("Failed to transmit: {}", ec.message());
    }
  }
  //! [sx126x example]
}
