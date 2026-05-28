#pragma once

#include "display_drivers.hpp"

namespace espp {
/**
 * @brief Display driver for the ILI9341 display controller.
 *
 * \section ili9341_wrover_cfg WROVER-KIT ILI9341 Config
 * \snippet display_drivers_example.cpp wrover_kit_config example
 * \section ili9341_ex1 ili9341 Example
 * \snippet display_drivers_example.cpp display_drivers example
 */
class Ili9341 : public display_drivers::MipiDbiDisplayDriver {
public:
  enum class Command : uint8_t {
    invoff = 0x20,
    invon = 0x21,
    gamset = 0x26,
    dispoff = 0x28,
    dispon = 0x29,
    caset = 0x2a,
    raset = 0x2b,
    ramwr = 0x2c,
    rgbset = 0x2d,
    ramrd = 0x2e,
    madctl = 0x36,
    idmoff = 0x38,
    idmon = 0x39,
    ramwrc = 0x3c,
    ramrdc = 0x3e,
    colmod = 0x3a,
  };

  explicit Ili9341(const display_drivers::Config &config)
      : MipiDbiDisplayDriver(config,
                             {.column_address_command = static_cast<uint8_t>(Command::caset),
                              .row_address_command = static_cast<uint8_t>(Command::raset),
                              .memory_write_command = static_cast<uint8_t>(Command::ramwr)}) {}

  bool initialize() override {
    display_drivers::init_pins(config_.reset_pin, config_.data_command_pin, config_.reset_value);

    auto madctl = make_madctl(DisplayRotation::LANDSCAPE);
    auto init_commands = std::to_array<display_drivers::DisplayInitCmd<>>({
        {0xCF, {0x00, 0x83, 0X30}},
        {0xED, {0x64, 0x03, 0X12, 0X81}},
        {0xE8, {0x85, 0x01, 0x79}},
        {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}},
        {0xF7, {0x20}},
        {0xEA, {0x00, 0x00}},
        {0xC0, {0x26}},
        {0xC1, {0x11}},
        {0xC5, {0x35, 0x3E}},
        {0xC7, {0xBE}},
        {0x36, {madctl}},
        {0x3A, {0x55}},
        {0xB1, {0x00, 0x1B}},
        {0xF2, {0x08}},
        {0x26, {0x01}},
        {0xE0,
         {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05,
          0x00}},
        {0XE1,
         {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A,
          0x1F}},
        {0x2A, {0x00, 0x00, 0x00, 0xEF}},
        {0x2B, {0x00, 0x00, 0x01, 0x3f}},
        {0x2C},
        {0xB7, {0x07}},
        {0xB6, {0x0A, 0x82, 0x27, 0x00}},
        {0x11, {}, 100},
        {0x29, {}, 100},
    });

    send_commands(init_commands);
    write_command(static_cast<uint8_t>(config_.invert_colors ? Command::invon : Command::invoff),
                  {}, 0);
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
