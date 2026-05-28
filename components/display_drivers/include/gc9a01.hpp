#pragma once

#include "display_drivers.hpp"

namespace espp {
/**
 * @brief Display driver for the GC9A01 display controller.
 *
 * \section smartknob_ha_cfg SmartKnob Config
 * \snippet display_drivers_example.cpp smartknob_config example
 * \section gc9a01_ex1 Gc9a01 Example
 * \snippet display_drivers_example.cpp display_drivers example
 */
class Gc9a01 : public display_drivers::MipiDbiDisplayDriver {
public:
  enum class Command : uint8_t {
    nop = 0x00,
    swreset = 0x01,
    rddid = 0x04,
    rddst = 0x09,
    slpin = 0x10,
    slpout = 0x11,
    ptlon = 0x12,
    noron = 0x13,
    invoff = 0x20,
    invon = 0x21,
    dispoff = 0x28,
    dispon = 0x29,
    caset = 0x2a,
    raset = 0x2b,
    ramwr = 0x2c,
    ptlar = 0x30,
    vscrdef = 0x33,
    teoff = 0x34,
    teon = 0x35,
    madctl = 0x36,
    idmoff = 0x38,
    idmon = 0x39,
    colmod = 0x3a,
    ramwrc = 0x3c,
    settes = 0x44,
    gettes = 0x45,
    wrdpbr = 0x51,
    wrctrldp = 0x53,
    readid1 = 0xDA,
    readid2 = 0xDB,
    readid3 = 0xDC,
    rgbctrl = 0xb0,
    porctrl = 0xb5,
    dpfuctrl = 0xb6,
    tectrl = 0xba,
    intrctrl = 0xf6,
    frctrl = 0xe8,
    spi2dctrl = 0xe9,
    pwrctrl1 = 0xc1,
    pwrctrl2 = 0xc3,
    pwrctrl3 = 0xc4,
    pwrctrl4 = 0xc9,
    pwrctrl7 = 0xa7,
    intren1 = 0xfe,
    intren2 = 0xef,
    stgamma1 = 0xf0,
    stgamma2 = 0xf1,
    stgamma3 = 0xf2,
    stgamma4 = 0xf3,
  };

  explicit Gc9a01(const display_drivers::Config &config)
      : MipiDbiDisplayDriver(config,
                             {.column_address_command = static_cast<uint8_t>(Command::caset),
                              .row_address_command = static_cast<uint8_t>(Command::raset),
                              .memory_write_command = static_cast<uint8_t>(Command::ramwr)}) {}

  bool initialize() override {
    display_drivers::init_pins(config_.reset_pin, config_.data_command_pin, config_.reset_value);

    auto madctl = make_madctl(DisplayRotation::LANDSCAPE);
    auto init_commands = std::to_array<display_drivers::DisplayInitCmd<>>({
        {0xEF},
        {0xEB, {0x14}},
        {0xFE},
        {0xEF},
        {0xEB, {0x14}},
        {0x84, {0x40}},
        {0x85, {0xFF}},
        {0x86, {0xFF}},
        {0x87, {0xFF}},
        {0x88, {0x0A}},
        {0x89, {0x21}},
        {0x8A, {0x00}},
        {0x8B, {0x80}},
        {0x8C, {0x01}},
        {0x8D, {0x01}},
        {0x8E, {0xFF}},
        {0x8F, {0xFF}},
        {0xB6, {0x00, 0x20}},
        {0x36, {madctl}},
        {0x3A, {0x05}},
        {0x90, {0x08, 0x08, 0X08, 0X08}},
        {0xBD, {0x06}},
        {0xBC, {0x00}},
        {0xFF, {0x60, 0x01, 0x04}},
        {0xC3, {0x13}},
        {0xC4, {0x13}},
        {0xC9, {0x22}},
        {0xBE, {0x11}},
        {0xE1, {0x10, 0x0E}},
        {0xDF, {0x21, 0x0C, 0x02}},
        {0xF0, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2A}},
        {0xF1, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6F}},
        {0xF2, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2A}},
        {0xF3, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6F}},
        {0xED, {0x1B, 0x0B}},
        {0xAE, {0x77}},
        {0xCD, {0x63}},
        {0x70, {0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0X08, 0x03}},
        {0xE8, {0x34}},
        {0x62, {0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0X0F, 0x71, 0xEF, 0x70, 0x70}},
        {0x63, {0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0X13, 0x71, 0xF3, 0x70, 0x70}},
        {0x64, {0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07}},
        {0x66, {0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0X00, 0x00, 0x00}},
        {0x67, {0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0X10, 0x32, 0x98}},
        {0x74, {0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00}},
        {0x98, {0x3E, 0x07}},
        {0x35},
        {0x21},
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
