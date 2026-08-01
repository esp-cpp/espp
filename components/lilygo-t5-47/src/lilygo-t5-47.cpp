#include "lilygo-t5-47.hpp"

namespace espp {

LilyGoT547::LilyGoT547()
    : BaseComponent("LilyGoT547", espp::Logger::Verbosity::INFO) {}

bool LilyGoT547::initialize_display() {
  if (initialized_) {
    logger_.warn("Display already initialized");
    return true;
  }
  logger_.info("Initializing ED047TC1 e-paper via epdiy");

  // Create the internal I2C bus ourselves and hand epdiy the handle, so the same
  // bus is shared with the board's other I2C peripherals (touch, RTC, battery
  // gauge, qwiic). epdiy's lilygo_board_s3 uses this bus (SDA=39/SCL=40, port 0,
  // 100 kHz) for the e-paper power ICs (PCA9555 + TPS65185); passing our handle
  // makes epdiy add its device handles to our bus instead of creating its own
  // (it sets owns_bus=false and leaves the bus alive for us).
  internal_i2c_ = std::make_unique<espp::I2c>(espp::I2c::Config{
      .port = I2C_NUM_0,
      .sda_io_num = internal_i2c_sda,
      .scl_io_num = internal_i2c_scl,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .clk_speed = 100 * 1000, // the board (and epdiy) run this bus at 100 kHz
      .log_level = espp::Logger::Verbosity::WARN,
  });
  auto i2c_bus_handle = internal_i2c_->native_bus_handle();
  if (i2c_bus_handle == nullptr) {
    logger_.error("Failed to initialize the internal I2C bus");
    internal_i2c_.reset();
    return false;
  }
  EpdI2cConfig epd_i2c_config = {.bus_handle = i2c_bus_handle};
  EpdInitConfig epd_init_config = {.i2c = &epd_i2c_config};

  // Bring up the parallel bus + waveform LUTs for this exact board/panel. Use
  // the ESP32-S3 board definition (lilygo_board_s3), which drives the panel via
  // the LCD_CAM peripheral. The original epd_board_lilygo_t5_47 is the ESP32
  // (I2S-parallel) board and pulls in the legacy I2S driver removed in IDF 6.
  epd_init_with_config(&lilygo_board_s3, &ED047TC1, EPD_LUT_64K, &epd_init_config);
  // Allocate the high-level framebuffer + use the built-in waveform.
  hl_ = epd_hl_init(EPD_BUILTIN_WAVEFORM);
  // Default to landscape (960 wide x 540 tall).
  epd_set_rotation(EPD_ROT_LANDSCAPE);

  logger_.info("Display initialized: {}x{}", width(), height());
  initialized_ = true;
  return true;
}

void LilyGoT547::power_on() {
  if (!powered_on_.exchange(true)) {
    epd_poweron();
  }
}

void LilyGoT547::power_off() {
  if (powered_on_.exchange(false)) {
    epd_poweroff();
  }
}

void LilyGoT547::clear() {
  if (!initialized_) {
    return;
  }
  bool was_off = !powered_on_;
  if (was_off) {
    power_on();
  }
  epd_fullclear(&hl_, temperature());
  if (was_off) {
    power_off();
  }
}

uint8_t *LilyGoT547::framebuffer() {
  if (!initialized_) {
    return nullptr;
  }
  return epd_hl_get_framebuffer(&hl_);
}

void LilyGoT547::update(EpdDrawMode mode) {
  if (!initialized_) {
    return;
  }
  bool was_off = !powered_on_;
  if (was_off) {
    power_on();
  }
  epd_hl_update_screen(&hl_, mode, temperature());
  if (was_off) {
    power_off();
  }
}

int LilyGoT547::temperature() {
  // The T5 4.7" has no dedicated temperature sensor wired for epdiy on all
  // revisions; use a reasonable room-temperature default. e-paper waveforms are
  // temperature-dependent, so if updates look wrong at temperature extremes this
  // is the value to feed from an external sensor.
  return 25;
}

} // namespace espp
