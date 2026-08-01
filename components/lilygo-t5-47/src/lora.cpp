#include "lilygo-t5-47.hpp"

#include <chrono>
#include <cstring>
#include <thread>

namespace espp {

bool LilyGoT547::initialize_lora(const espp::Sx126x::RadioConfig &radio_config) {
  if (lora_) {
    logger_.warn("LoRa radio already initialized, not initializing again!");
    return false;
  }

  // The LoRa (and GPS) module is power-gated by the PCA9535 expander pin P00
  // (see the vendor firmware's io_extend_lora_gps_power_on()). Without driving it
  // high the SX1262 is unpowered and does not answer on SPI. epdiy only uses the
  // expander's port 1, so driving port-0 bit 0 is safe.
  if (!io_expander_ && !initialize_io_expander()) {
    logger_.error("Could not initialize the I/O expander to power the LoRa module");
    return false;
  }
  {
    static constexpr uint8_t P00 = 0x01; // port 0, bit 0 = LoRa/GPS power enable
    std::error_code ec;
    uint8_t dir = io_expander_->get_direction(espp::Pca9535::Port::PORT0, ec);
    io_expander_->set_direction(espp::Pca9535::Port::PORT0, dir & ~P00, ec); // P00 -> output
    uint8_t out = io_expander_->get_output(espp::Pca9535::Port::PORT0, ec);
    io_expander_->set_pins(espp::Pca9535::Port::PORT0, out | P00, ec); // P00 -> high (on)
    if (ec) {
      logger_.warn("Could not enable LoRa power via the expander (P00): {}", ec.message());
    } else {
      logger_.info("Enabled LoRa/GPS power (expander P00)");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // let the module power up
  }

  // The radio shares the SPI bus with the microSD card; make sure it is up.
  if (!init_spi_bus()) {
    logger_.error("Failed to initialize SPI bus for the LoRa radio");
    return false;
  }

  // The DIO1 interrupt is serviced by the shared interrupt manager, which needs
  // epdiy to have installed the GPIO ISR service first.
  if (!ensure_interrupts()) {
    return false;
  }

  logger_.info("Initializing LoRa radio (SX1262, CS={})", static_cast<int>(lora_cs_io));

  std::error_code ec;
  lora_spi_device_ = spi_->add_device(
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

  // Configure the radio's control pins.
  gpio_set_direction(lora_busy_io, GPIO_MODE_INPUT);
  gpio_set_direction(lora_reset_io, GPIO_MODE_OUTPUT);
  gpio_set_level(lora_reset_io, 1);

  auto device = lora_spi_device_;
  auto write_fn = [device](const uint8_t *data, size_t length) -> bool {
    std::error_code ec;
    return device->write(std::span{data, length}, {}, ec);
  };
  // The SX126x holds chip-select asserted across a command's write and read
  // phases, so write_then_read is a single full-duplex transfer of the
  // concatenated length. Use fixed DMA-aligned stack buffers (this is the hot
  // path for every command / IRQ read); the largest transfer is a 255-byte FIFO
  // read plus a few opcode/param bytes.
  auto write_then_read_fn = [device](const uint8_t *write_data, size_t write_length,
                                     uint8_t *read_data, size_t read_length) -> bool {
    static constexpr size_t kMaxTransfer = 264;
    const size_t total = write_length + read_length;
    if (total > kMaxTransfer) {
      return false;
    }
    alignas(4) uint8_t tx[kMaxTransfer] = {0};
    alignas(4) uint8_t rx[kMaxTransfer] = {0};
    std::memcpy(tx, write_data, write_length);
    std::error_code ec;
    if (!device->transfer(std::span{tx, total}, std::span{rx, total}, {}, ec)) {
      return false;
    }
    std::memcpy(read_data, rx + write_length, read_length);
    return true;
  };

  lora_ = std::make_shared<espp::Sx126x>(espp::Sx126x::Config{
      .variant = espp::Sx126x::Variant::SX1262,
      .write = write_fn,
      .write_then_read = write_then_read_fn,
      .is_busy = []() -> bool { return gpio_get_level(lora_busy_io); },
      .reset = [](bool level) { gpio_set_level(lora_reset_io, level); },
      // The module has a TCXO powered from DIO3 (2.4 V) and uses DIO2 to control
      // its RF switch (matching the vendor firmware); it uses the LDO regulator.
      .tcxo_voltage = 2.4f,
      .use_dio2_as_rf_switch = true,
      .use_dcdc_regulator = false,
      .radio_config = radio_config,
      .auto_init = false,
      .log_level = get_log_level()});
  if (!lora_->initialize(ec)) {
    logger_.error("Failed to initialize LoRa radio: {}", ec.message());
    lora_.reset();
    lora_spi_device_.reset();
    return false;
  }

  // Service the radio's IRQs whenever DIO1 goes high.
  interrupts_->add_interrupt(lora_dio1_interrupt_pin_);

  return true;
}

} // namespace espp
