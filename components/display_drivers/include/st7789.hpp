#pragma once

#include "display_drivers.hpp"

namespace espp {
/**
 * @brief Display driver for the ST7789 display controller.
 *
 *   This code is modified from
 *   https://github.com/lvgl/lvgl_esp32_drivers/blob/master/lvgl_tft/st7789.c
 *   and
 *   https://github.com/Bodmer/TFT_eSPI/blob/master/TFT_Drivers/ST7789_Defines.h
 *
 * \section st7789_ttgo_cfg TTGO St7789 Config
 * \snippet display_drivers_example.cpp ttgo_config example
 * \section st7789_box_cfg ESP32-S3-BOX St7789 Config
 * \snippet display_drivers_example.cpp box_config example
 * \section st7789_ex1 st7789 Example
 * \snippet display_drivers_example.cpp display_drivers example
 */
class St7789 : public display_drivers::MipiDbiDisplayDriver {
public:
  enum class Command : uint8_t {
    nop = 0x00,
    swreset = 0x01,
    rddid = 0x04,
    rddst = 0x09,
    rddpm = 0x0a,
    rdd_madctl = 0x0b,
    rdd_colmod = 0x0c,
    rddim = 0x0d,
    rddsm = 0x0e,
    rddsr = 0x0f,
    slpin = 0x10,
    slpout = 0x11,
    ptlon = 0x12,
    noron = 0x13,
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
    ptlar = 0x30,
    vscrdef = 0x33,
    teoff = 0x34,
    teon = 0x35,
    madctl = 0x36,
    idmoff = 0x38,
    idmon = 0x39,
    ramwrc = 0x3c,
    ramrdc = 0x3e,
    colmod = 0x3a,
    ramctrl = 0xb0,
    rgbctrl = 0xb1,
    porctrl = 0xb2,
    frctrl1 = 0xb3,
    parctrl = 0xb5,
    gctrl = 0xb7,
    gtadj = 0xb8,
    dgmen = 0xba,
    vcoms = 0xbb,
    lcmctrl = 0xc0,
    idset = 0xc1,
    vdvvrhen = 0xc2,
    vrhs = 0xc3,
    vdvset = 0xc4,
    vcmofset = 0xc5,
    frctr2 = 0xc6,
    cabcctrl = 0xc7,
    regsel1 = 0xc8,
    regsel2 = 0xca,
    pwmfrsel = 0xcc,
    pwctrl1 = 0xd0,
    vapvanen = 0xd2,
    cmd2en = 0xdf,
    pvgamctrl = 0xe0,
    nvgamctrl = 0xe1,
    dgmlutr = 0xe2,
    dgmlutb = 0xe3,
    gatectrl = 0xe4,
    spi2en = 0xe7,
    pwctrl2 = 0xe8,
    eqctrl = 0xe9,
    promctrl = 0xec,
    promen = 0xfa,
    nvmset = 0xfc,
    promact = 0xfe,
  };

  explicit St7789(const display_drivers::Config &config)
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
        {static_cast<uint8_t>(Command::pwctrl2), {0x85, 0x01, 0x79}},
        {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}},
        {0xF7, {0x20}},
        {0xEA, {0x00, 0x00}},
        {static_cast<uint8_t>(Command::lcmctrl), {0x26}},
        {static_cast<uint8_t>(Command::idset), {0x11}},
        {static_cast<uint8_t>(Command::vcmofset), {0x35, 0x3E}},
        {static_cast<uint8_t>(Command::cabcctrl), {0xBE}},
        {static_cast<uint8_t>(Command::madctl), {madctl}},
        {static_cast<uint8_t>(Command::colmod), {0x55}},
        {static_cast<uint8_t>(Command::invon)},
        {static_cast<uint8_t>(Command::rgbctrl), {0x00, 0x1B}},
        {0xF2, {0x08}},
        {static_cast<uint8_t>(Command::gamset), {0x01}},
        {static_cast<uint8_t>(Command::caset), {0x00, 0x00, 0x00, 0xEF}},
        {static_cast<uint8_t>(Command::raset), {0x00, 0x00, 0x01, 0x3f}},
        {static_cast<uint8_t>(Command::ramwr)},
        {static_cast<uint8_t>(Command::gctrl), {0x07}},
        {0xB6, {0x0A, 0x82, 0x27, 0x00}},
        {static_cast<uint8_t>(Command::slpout), {0}, 100},
        {static_cast<uint8_t>(Command::dispon), {0}, 100},
    });

    if (config_.invert_colors) {
      init_commands[12].command = static_cast<uint8_t>(Command::invoff);
    }

    send_commands(init_commands);
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
