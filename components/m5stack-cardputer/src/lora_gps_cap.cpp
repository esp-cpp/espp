#include "m5stack-cardputer.hpp"

#include <cstring>

using namespace espp;

/////////////////////////////////////////////////////////////////////////////
// LoRa + GPS Cap (U201 / U214 expansion module for the Cardputer ADV)
/////////////////////////////////////////////////////////////////////////////

bool M5StackCardputer::ensure_expansion_spi_bus() {
  if (expansion_spi_bus_initialized_) {
    return true;
  }
  spi_bus_config_t bus_cfg;
  memset(&bus_cfg, 0, sizeof(bus_cfg));
  bus_cfg.mosi_io_num = sdcard_mosi;
  bus_cfg.miso_io_num = sdcard_miso;
  bus_cfg.sclk_io_num = sdcard_sclk;
  bus_cfg.quadwp_io_num = -1;
  bus_cfg.quadhd_io_num = -1;
  bus_cfg.max_transfer_sz = 4092;

  auto ret = spi_bus_initialize(sdcard_spi_num, &bus_cfg, SDSPI_DEFAULT_DMA);
  if (ret != ESP_OK) {
    logger_.error("Failed to initialize expansion SPI bus: {}", esp_err_to_name(ret));
    return false;
  }
  expansion_spi_bus_initialized_ = true;
  return true;
}

bool M5StackCardputer::enable_cap_expander_outputs(uint8_t mask) {
  // The U214 Cap has a PI4IOE5V6408 IO expander on the internal I2C bus
  // controlling the LoRa RF switch (P0) and the GPS LNA (P1). The older U201
  // Cap has no expander, in which case this is a no-op.
  auto *i2c = internal_i2c();
  if (!i2c) {
    logger_.warn("No internal I2C bus (original Cardputer?); cannot configure the Cap's IO "
                 "expander - the LoRa+GPS Cap is designed for the Cardputer ADV");
    return false;
  }
  if (!i2c->probe_device(cap_expander_address)) {
    logger_.info("No IO expander found on the Cap (older U201 Cap?); assuming the RF switch / "
                 "GPS LNA are hardwired");
    return true;
  }
  // registers: 0x03 = IO direction (1 = output), 0x05 = output state,
  // 0x07 = output high-impedance (1 = Hi-Z, the power-on default)
  const uint8_t reg_direction = 0x03;
  const uint8_t reg_output = 0x05;
  const uint8_t reg_high_z = 0x07;
  auto update_register = [&](uint8_t reg, uint8_t set_mask, uint8_t clear_mask) -> bool {
    uint8_t value = 0;
    if (!i2c->write_read(cap_expander_address, &reg, 1, &value, 1)) {
      return false;
    }
    value = (value | set_mask) & ~clear_mask;
    const uint8_t data[2] = {reg, value};
    return i2c->write(cap_expander_address, data, 2);
  };
  if (!update_register(reg_direction, mask, 0) || !update_register(reg_output, mask, 0) ||
      !update_register(reg_high_z, 0, mask)) {
    logger_.error("Failed to configure the Cap's IO expander");
    return false;
  }
  return true;
}

bool M5StackCardputer::initialize_lora(const Sx126x::RadioConfig &radio_config) {
  if (lora_) {
    logger_.warn("LoRa radio already initialized, not initializing again!");
    return false;
  }

  // ensure the (shared) SPI bus is initialized
  if (!ensure_expansion_spi_bus()) {
    return false;
  }

  logger_.info("Initializing LoRa radio");

  // connect the antenna (via the Cap's IO expander, if present). A false return
  // here means the expander is present but could not be configured (a hardwired
  // Cap with no expander returns true), so the RF switch may be left Hi-Z and
  // the antenna disconnected - warn loudly but continue so the radio still comes
  // up for diagnostics.
  if (!enable_cap_expander_outputs(cap_expander_lora_rf_switch_mask)) {
    logger_.warn("Failed to enable the LoRa RF switch on the Cap's IO expander; the antenna may "
                 "be disconnected and range will be severely degraded");
  }

  // add the radio to the SPI bus
  spi_device_interface_config_t dev_cfg;
  memset(&dev_cfg, 0, sizeof(dev_cfg));
  dev_cfg.mode = 0;
  dev_cfg.clock_speed_hz = lora_spi_clock_speed;
  dev_cfg.spics_io_num = lora_cs_io;
  dev_cfg.queue_size = 1;
  auto ret = spi_bus_add_device(sdcard_spi_num, &dev_cfg, &lora_spi_handle_);
  if (ret != ESP_OK) {
    logger_.error("Failed to add LoRa radio to SPI bus: {}", esp_err_to_name(ret));
    return false;
  }

  // configure the radio's control pins
  gpio_set_direction(lora_busy_io, GPIO_MODE_INPUT);
  gpio_set_direction(lora_reset_io, GPIO_MODE_OUTPUT);
  gpio_set_level(lora_reset_io, 1);

  // The SX126x requires chip select to be held asserted across the write and
  // read phases of a command, so write_then_read is implemented as a single
  // full-duplex transfer of the concatenated length.
  auto handle = lora_spi_handle_;
  auto write_fn = [handle](const uint8_t *data, size_t length) -> bool {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = length * 8;
    t.tx_buffer = data;
    return spi_device_polling_transmit(handle, &t) == ESP_OK;
  };
  auto write_then_read_fn = [handle](const uint8_t *write_data, size_t write_length,
                                     uint8_t *read_data, size_t read_length) -> bool {
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
    alignas(4) uint8_t rx[kMaxTransfer] = {0};
    memcpy(tx, write_data, write_length);
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = total * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    if (spi_device_polling_transmit(handle, &t) != ESP_OK) {
      return false;
    }
    memcpy(read_data, rx + write_length, read_length);
    return true;
  };

  std::error_code ec;
  lora_ = std::make_shared<Sx126x>(
      Sx126x::Config{.variant = Sx126x::Variant::SX1262,
                     .write = write_fn,
                     .write_then_read = write_then_read_fn,
                     .is_busy = []() -> bool { return gpio_get_level(lora_busy_io); },
                     .reset = [](bool level) { gpio_set_level(lora_reset_io, level); },
                     // The Stamp LoRa-1262 Mini module has a TCXO powered from
                     // DIO3 at 3.0V and uses the LDO regulator (matching
                     // M5Stack's own RadioLib example: begin(..., 3.0, true)).
                     // Without powering the TCXO the oscillator never starts,
                     // so the radio can talk over SPI but cannot transmit or
                     // receive. The RF switch is driven by the Cap's IO
                     // expander (not DIO2), but enabling DIO2 control is
                     // harmless as it is unconnected on this module.
                     .tcxo_voltage = 3.0f,
                     .use_dio2_as_rf_switch = true,
                     .use_dcdc_regulator = false,
                     .radio_config = radio_config,
                     .auto_init = false,
                     .log_level = get_log_level()});
  if (!lora_->initialize(ec)) {
    logger_.error("Failed to initialize LoRa radio: {}", ec.message());
    lora_.reset();
    spi_bus_remove_device(lora_spi_handle_);
    lora_spi_handle_ = nullptr;
    return false;
  }

  // service the radio's IRQs whenever DIO1 goes high
  interrupts_.add_interrupt(lora_dio1_interrupt_pin_);

  return true;
}

bool M5StackCardputer::initialize_gps(const Gps::fix_callback_fn &fix_cb, uint32_t baud_rate) {
  if (gps_) {
    logger_.warn("GPS already initialized, not initializing again!");
    return false;
  }

  logger_.info("Initializing GPS");

  // enable the GPS LNA (via the Cap's IO expander, if present)
  enable_cap_expander_outputs(cap_expander_gps_lna_mask);

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
