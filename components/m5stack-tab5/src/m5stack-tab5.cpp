#include "m5stack-tab5.hpp"

#include <driver/gpio.h>
#include <esp_check.h>
#include <esp_heap_caps.h>
#include <esp_log.h>

// Display, touch, audio, camera, and IMU implementations are split into
// separate compilation units to match the esp-box structure.

namespace espp {

M5StackTab5::M5StackTab5()
    : BaseComponent("M5StackTab5") {}

bool M5StackTab5::initialize_io_expanders() {
  logger_.info("Initializing IO expanders (0x43, 0x44)");
  std::error_code ec;

  // Create instances
  ioexp_0x43_ = std::make_unique<Pi4ioe5v>(Pi4ioe5v::Config{
      .device_address = 0x43,
      .probe = std::bind(&I2c::probe_device, &internal_i2c_, std::placeholders::_1),
      .write = std::bind(&I2c::write, &internal_i2c_, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3),
      .read_register =
          std::bind(&I2c::read_at_register, &internal_i2c_, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
      .write_then_read =
          std::bind(&I2c::write_read, &internal_i2c_, std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
      .auto_init = false,
      .log_level = Logger::Verbosity::WARN});
  ioexp_0x44_ = std::make_unique<Pi4ioe5v>(Pi4ioe5v::Config{
      .device_address = 0x44,
      .probe = std::bind(&I2c::probe_device, &internal_i2c_, std::placeholders::_1),
      .write = std::bind(&I2c::write, &internal_i2c_, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3),
      .read_register =
          std::bind(&I2c::read_at_register, &internal_i2c_, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
      .write_then_read =
          std::bind(&I2c::write_read, &internal_i2c_, std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
      .auto_init = false,
      .log_level = Logger::Verbosity::WARN});

  // Configure 0x43 using IOX_0x43_* sets
  {
    auto &io = *ioexp_0x43_;
    uint8_t dir = 0xFF;
    for (int i = 0; i < IOX_0x43_OUTPUTS_COUNT; ++i)
      dir &= ~(1u << IOX_0x43_OUTPUTS[i]);
    for (int i = 0; i < IOX_0x43_INPUTS_COUNT; ++i)
      dir |= (1u << IOX_0x43_INPUTS[i]);
    io.set_direction(dir, ec);
    if (ec) {
      logger_.error("ioexp 0x43 set_direction failed: {}", ec.message());
      return false;
    }
    uint8_t out = 0; // default all outputs to low
    io.write_outputs(out, ec);
    if (ec) {
      logger_.error("ioexp 0x43 write_outputs failed: {}", ec.message());
      return false;
    }
  }

  // Configure 0x44 using IOX_0x44_* sets
  {
    auto &io = *ioexp_0x44_;
    uint8_t dir = 0xFF;
    for (int i = 0; i < IOX_0x44_OUTPUTS_COUNT; ++i)
      dir &= ~(1u << IOX_0x44_OUTPUTS[i]);
    for (int i = 0; i < IOX_0x44_INPUTS_COUNT; ++i)
      dir |= (1u << IOX_0x44_INPUTS[i]);
    io.set_direction(dir, ec);
    if (ec) {
      logger_.error("ioexp 0x44 set_direction failed: {}", ec.message());
      return false;
    }
    // Safe defaults: disable charging, USB 5V off, WLAN power off, PWROFF pulse low
    uint8_t out = 0x00;
    io.write_outputs(out, ec);
    if (ec) {
      logger_.error("ioexp 0x44 write_outputs failed: {}", ec.message());
      return false;
    }
  }

  logger_.info("IO expanders initialized");
  return true;
}

void M5StackTab5::lcd_reset(bool assert_reset) {
  set_io_expander_output(0x43, IO43_BIT_LCD_RST, !assert_reset);
}

void M5StackTab5::touch_reset(bool assert_reset) {
  set_io_expander_output(0x43, IO43_BIT_TP_RST, !assert_reset);
}

void M5StackTab5::set_speaker_enabled(bool enable) {
  set_io_expander_output(0x43, IO43_BIT_SPK_EN, enable);
}

void M5StackTab5::set_charging_enabled(bool enable) {
  set_io_expander_output(0x44, IO44_BIT_CHG_EN, enable);
}

bool M5StackTab5::charging_status() {
  auto state = get_io_expander_input(0x44, IO44_BIT_CHG_STAT);
  return state.value_or(false);
}

bool M5StackTab5::set_io_expander_output(uint8_t address, uint8_t bit, bool level) {
  std::error_code ec;
  espp::Pi4ioe5v *io = nullptr;
  if (address == 0x43)
    io = ioexp_0x43_.get();
  else if (address == 0x44)
    io = ioexp_0x44_.get();
  if (!io) {
    // Try lazy initialization
    if (!initialize_io_expanders())
      return false;
    if (address == 0x43)
      io = ioexp_0x43_.get();
    else if (address == 0x44)
      io = ioexp_0x44_.get();
  }
  if (!io)
    return false;
  uint8_t val = io->read_outputs(ec);
  if (ec)
    return false;
  if (level)
    val |= (1u << bit);
  else
    val &= ~(1u << bit);
  io->write_outputs(val, ec);
  return !ec;
}

std::optional<bool> M5StackTab5::get_io_expander_output(uint8_t address, uint8_t bit) {
  std::error_code ec;
  espp::Pi4ioe5v *io = nullptr;
  if (address == 0x43)
    io = ioexp_0x43_.get();
  else if (address == 0x44)
    io = ioexp_0x44_.get();
  if (!io) {
    // Try lazy initialization
    const_cast<M5StackTab5 *>(this)->initialize_io_expanders();
    if (address == 0x43)
      io = ioexp_0x43_.get();
    else if (address == 0x44)
      io = ioexp_0x44_.get();
  }
  if (!io)
    return std::nullopt;
  uint8_t val = io->read_outputs(ec);
  if (ec)
    return std::nullopt;
  return (val >> bit) & 0x1u;
}

std::optional<bool> M5StackTab5::get_io_expander_input(uint8_t address, uint8_t bit) {
  std::error_code ec;
  espp::Pi4ioe5v *io = nullptr;
  if (address == 0x43)
    io = ioexp_0x43_.get();
  else if (address == 0x44)
    io = ioexp_0x44_.get();
  if (!io) {
    // Try lazy initialization
    const_cast<M5StackTab5 *>(this)->initialize_io_expanders();
    if (address == 0x43)
      io = ioexp_0x43_.get();
    else if (address == 0x44)
      io = ioexp_0x44_.get();
  }
  if (!io)
    return std::nullopt;
  uint8_t val = io->read_inputs(ec);
  if (ec)
    return std::nullopt;
  return (val >> bit) & 0x1u;
}

void M5StackTab5::set_backlight_enabled(bool enable) {
  // If backlight is wired through expander in future, route here.
  // For now, drive the local GPIO.
  gpio_set_level(lcd_backlight_io, enable ? 1 : 0);
}

std::optional<bool> M5StackTab5::is_backlight_enabled() const {
  return gpio_get_level(lcd_backlight_io) != 0;
}

// (Video/display, touch, audio, camera, and IMU are implemented in
// video.cpp, touchpad.cpp, audio.cpp, camera.cpp, and imu.cpp respectively.)

} // namespace espp
