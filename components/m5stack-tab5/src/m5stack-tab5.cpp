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
  // Create instances
  ioexp_0x43_ = std::make_shared<IoExpander>(IoExpander::Config{
      .device_address = 0x43,
      .direction_mask = IOX_0x43_DIRECTION_MASK,
      .initial_output = IOX_0x43_DEFAULT_OUTPUTS,
      .high_z_mask = IOX_0x43_HIGH_Z_MASK,
      .pull_up_mask = IOX_0x43_PULL_UPS,
      .pull_down_mask = IOX_0x43_PULL_DOWNS,
      .write = std::bind(&I2c::write, &internal_i2c_, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3),
      .write_then_read =
          std::bind(&I2c::write_read, &internal_i2c_, std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
      .log_level = Logger::Verbosity::INFO});
  ioexp_0x44_ = std::make_shared<IoExpander>(IoExpander::Config{
      .device_address = 0x44,
      .direction_mask = IOX_0x44_DIRECTION_MASK,
      .initial_output = IOX_0x44_DEFAULT_OUTPUTS,
      .high_z_mask = IOX_0x44_HIGH_Z_MASK,
      .pull_up_mask = IOX_0x44_PULL_UPS,
      .pull_down_mask = IOX_0x44_PULL_DOWNS,
      .write = std::bind(&I2c::write, &internal_i2c_, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3),
      .write_then_read =
          std::bind(&I2c::write_read, &internal_i2c_, std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
      .log_level = Logger::Verbosity::INFO});

  logger_.info("IO expanders initialized");
  return true;
}

bool M5StackTab5::lcd_reset(bool assert_reset) {
  return set_io_expander_output(0x43, IO43_BIT_LCD_RST, !assert_reset);
}

bool M5StackTab5::touch_reset(bool assert_reset) {
  return set_io_expander_output(0x43, IO43_BIT_TP_RST, !assert_reset);
}

bool M5StackTab5::set_speaker_enabled(bool enable) {
  return set_io_expander_output(0x43, IO43_BIT_SPK_EN, enable);
}

bool M5StackTab5::set_charging_enabled(bool enable) {
  return set_io_expander_output(0x44, IO44_BIT_CHG_EN, enable);
}

bool M5StackTab5::get_charging_status() {
  auto state = get_io_expander_input(0x44, IO44_BIT_CHG_STAT);
  return state.value_or(false);
}

bool M5StackTab5::set_io_expander_output(uint8_t address, uint8_t bit, bool level) {
  std::error_code ec;
  IoExpander *io = nullptr;
  if (address == 0x43)
    io = ioexp_0x43_.get();
  else if (address == 0x44)
    io = ioexp_0x44_.get();
  if (!io)
    return false;
  if (level)
    return io->set_pins(1u << bit, ec);
  else
    return io->clear_pins(1u << bit, ec);
}

std::optional<bool> M5StackTab5::get_io_expander_output(uint8_t address, uint8_t bit) {
  std::error_code ec;
  IoExpander *io = nullptr;
  if (address == 0x43)
    io = ioexp_0x43_.get();
  else if (address == 0x44)
    io = ioexp_0x44_.get();
  if (!io)
    return std::nullopt;
  uint8_t val = io->get_output(ec);
  if (ec)
    return std::nullopt;
  return (val >> bit) & 0x1u;
}

std::optional<bool> M5StackTab5::get_io_expander_input(uint8_t address, uint8_t bit) {
  std::error_code ec;
  IoExpander *io = nullptr;
  if (address == 0x43)
    io = ioexp_0x43_.get();
  else if (address == 0x44)
    io = ioexp_0x44_.get();
  if (!io)
    return std::nullopt;
  uint8_t val = io->get_input(ec);
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
