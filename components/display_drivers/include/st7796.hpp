#pragma once

#include <algorithm>
#include <array>
#include <mutex>
#include <thread>

#include "display_drivers.hpp"

namespace espp {
/**
 * @brief Display driver for the ST7796 display controller.
 */
class St7796 : public display_drivers::MipiDbiDisplayDriver {
public:
  /// Supported command values used by this driver wrapper.
  enum class Command : uint8_t {
    slpout = 0x11, ///< Exit sleep mode.
    invoff = 0x20, ///< Disable display inversion.
    invon = 0x21,  ///< Enable display inversion.
    dispon = 0x29, ///< Turn the display on.
    caset = 0x2a,  ///< Set the column address window.
    raset = 0x2b,  ///< Set the row address window.
    ramwr = 0x2c,  ///< Write pixel data to RAM.
    madctl = 0x36, ///< Configure memory addressing and rotation.
    colmod = 0x3a, ///< Configure color depth / pixel format.
  };

  explicit St7796(const display_drivers::Config &config)
      : MipiDbiDisplayDriver(config,
                             {.column_address_command = static_cast<uint8_t>(Command::caset),
                              .row_address_command = static_cast<uint8_t>(Command::raset),
                              .memory_write_command = static_cast<uint8_t>(Command::ramwr)}) {}

  bool initialize() override {
    display_drivers::init_pins(config_.reset_pin, config_.data_command_pin, config_.reset_value);

    auto madctl = make_madctl(DisplayRotation::LANDSCAPE);
    auto init_commands = std::to_array<display_drivers::DisplayInitCmd<>>({
        {(uint8_t)Command::slpout, {}, 120},
        {(uint8_t)Command::madctl, {madctl}},
        {(uint8_t)Command::colmod, {0x55}},
        {0xf0, {0xc3}},
        {0xf0, {0x96}},
        {0xb4, {0x01}},
        {0xb5, {0x1e}},
        {0xb6, {0x80, 0x22, 0x3b}},
        {0xb7, {0xc6}},
        {0xb9, {0x02, 0xe0}},
        {0xc0, {0x80, 0x16}},
        {0xc1, {0x19}},
        {0xc2, {0xa7}},
        {0xc5, {0x16}},
        {0xe8, {0x40, 0x8a, 0x00, 0x00, 0x29, 0x19, 0xa5, 0x33}},
        {0xe0,
         {0xf0, 0x07, 0x0d, 0x04, 0x05, 0x14, 0x36, 0x54, 0x4c, 0x38, 0x13, 0x14, 0x2e, 0x34}},
        {0xe1,
         {0xf0, 0x10, 0x14, 0x0e, 0x0c, 0x08, 0x35, 0x44, 0x4c, 0x26, 0x10, 0x12, 0x2c, 0x32}},
        {0xf0, {0x3c}},
        {0xf0, {0x69}, 120},
        {(uint8_t)Command::dispon},
    });

    send_commands(init_commands);

    if (config_.invert_colors) {
      write_command(static_cast<uint8_t>(Command::invon), {}, 0);
    } else {
      write_command(static_cast<uint8_t>(Command::invoff), {}, 0);
    }
    return true;
  }

  void set_rotation(const DisplayRotation &rotation) override {
    Controller::set_rotation(rotation);
    auto data = std::array<uint8_t, 1>{make_madctl(rotation)};
    std::scoped_lock lock(io_mutex_);
    write_command(static_cast<uint8_t>(Command::madctl), data, 0);
  }

private:
  uint8_t make_madctl(DisplayRotation rotation) const {
    auto value = display_drivers::make_madctl_base(config_, LCD_CMD_BGR_BIT, LCD_CMD_MX_BIT,
                                                   LCD_CMD_MY_BIT, LCD_CMD_MV_BIT);
    return display_drivers::apply_standard_rotation(value, config_, rotation, LCD_CMD_MX_BIT,
                                                    LCD_CMD_MY_BIT, LCD_CMD_MV_BIT);
  }
};
} // namespace espp
