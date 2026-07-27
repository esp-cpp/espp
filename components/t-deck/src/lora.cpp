#include "t-deck.hpp"

#include <cstring>

using namespace espp;

/////////////////////////////////////////////////////////////////////////////
// LoRa Radio (SX1262, HPD16A module)
/////////////////////////////////////////////////////////////////////////////

bool TDeck::initialize_lora(const Sx126x::RadioConfig &radio_config) {
  if (lora_) {
    logger_.warn("LoRa radio already initialized, not initializing again!");
    return false;
  }

  if (microphone_initialized_) {
    // GPIO 17 is the LoRa radio reset line and is also routed to the T-Deck's
    // (unused-by-espp) PDM digital-microphone clock / ES7210 interrupt line.
    // The espp microphone path uses the ES7210 array on a separate set of
    // pins (it does not drive GPIO 17), so using the radio and the microphone
    // together is generally fine - warn rather than fail.
    logger_.warn("The LoRa radio reset line (GPIO 17) is shared with the microphone interrupt / "
                 "PDM dmic clock line; this is usually safe with the ES7210 microphone path, but "
                 "avoid enabling a PDM microphone at the same time.");
  }

  // ensure that the (shared) SPI bus is initialized
  if (!init_spi_bus()) {
    logger_.error("Failed to initialize SPI bus.");
    return false;
  }

  logger_.info("Initializing LoRa radio");

  std::error_code ec;
  lora_spi_device_ = lcd_spi_->add_device(
      {
          .mode = 0,
          .clock_speed_hz = lora_spi_clock_speed,
          .cs_io_num = lora_cs_io,
          .queue_size = spi_queue_size,
      },
      ec);
  if (!lora_spi_device_) {
    logger_.error("Failed to add LoRa radio to SPI bus: {}", ec.message());
    return false;
  }

  // configure the radio's control pins
  gpio_set_direction(lora_busy_io, GPIO_MODE_INPUT);
  gpio_set_direction(lora_reset_io, GPIO_MODE_OUTPUT);
  gpio_set_level(lora_reset_io, 1);

  // The SX126x requires chip select to be held asserted across the write and
  // read phases of a command, so write_then_read is implemented as a single
  // full-duplex transfer of the concatenated length.
  auto device = lora_spi_device_;
  auto write_fn = [device](const uint8_t *data, size_t length) -> bool {
    std::error_code ec;
    return device->write(std::span{data, length}, {}, ec);
  };
  auto write_then_read_fn = [device](const uint8_t *write_data, size_t write_length,
                                     uint8_t *read_data, size_t read_length) -> bool {
    std::vector<uint8_t> tx(write_length + read_length, 0);
    std::vector<uint8_t> rx(write_length + read_length, 0);
    std::memcpy(tx.data(), write_data, write_length);
    std::error_code ec;
    if (!device->transfer(tx, rx, {}, ec)) {
      return false;
    }
    std::memcpy(read_data, rx.data() + write_length, read_length);
    return true;
  };

  lora_ = std::make_shared<Sx126x>(
      Sx126x::Config{.variant = Sx126x::Variant::SX1262,
                     .write = write_fn,
                     .write_then_read = write_then_read_fn,
                     .is_busy = []() -> bool { return gpio_get_level(lora_busy_io); },
                     .reset = [](bool level) { gpio_set_level(lora_reset_io, level); },
                     // the HPD16A module has a TCXO powered from DIO3 at 1.8V,
                     // and uses DIO2 to control its RF switch
                     .tcxo_voltage = 1.8f,
                     .use_dio2_as_rf_switch = true,
                     .use_dcdc_regulator = true,
                     .radio_config = radio_config,
                     .auto_init = false,
                     .log_level = get_log_level()});
  if (!lora_->initialize(ec)) {
    logger_.error("Failed to initialize LoRa radio: {}", ec.message());
    lora_.reset();
    lora_spi_device_.reset();
    return false;
  }

  // service the radio's IRQs whenever DIO1 goes high
  interrupts_.add_interrupt(lora_dio1_interrupt_pin_);

  return true;
}

/////////////////////////////////////////////////////////////////////////////
// GPS (T-Deck Plus only, u-blox MIA-M10Q)
/////////////////////////////////////////////////////////////////////////////

bool TDeck::initialize_gps(const Gps::fix_callback_fn &fix_cb, uint32_t baud_rate) {
  if (gps_) {
    logger_.warn("GPS already initialized, not initializing again!");
    return false;
  }

  logger_.info("Initializing GPS");

  gps_ = std::make_shared<Gps>(Gps::Config{.uart_port = gps_uart_port,
                                           .tx_io_num = gps_tx_io,
                                           .rx_io_num = gps_rx_io,
                                           .baud_rate = baud_rate,
                                           .on_fix = fix_cb,
                                           .auto_start = false,
                                           .log_level = get_log_level()});
  std::error_code ec;
  if (!gps_->start(ec)) {
    logger_.error("Failed to start GPS: {}", ec.message());
    gps_.reset();
    return false;
  }
  return true;
}
