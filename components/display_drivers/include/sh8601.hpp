#pragma once

#include "display_drivers.hpp"

namespace espp {
/**
 * @brief Display driver for the SH8601 display controller.
 *
 * \section t_encoder_pro_cfg SmartKnob Config
 * \snippet display_drivers_example.cpp t_encoder_pro_config example
 * \section sh8601_ex1 Sh8601 Example
 * \snippet display_drivers_example.cpp display_drivers example
 */
class Sh8601 : public display_drivers::MipiDbiDisplayDriver {
public:
  enum class TransferMode : uint8_t {
    SINGLE_LINE = 0x02,
    MULTI_LINE = 0x32,
  };

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
    paset = 0x2b,
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

  explicit Sh8601(const display_drivers::Config &config)
      : MipiDbiDisplayDriver(config,
                             {.column_address_command = static_cast<uint8_t>(Command::caset),
                              .row_address_command = static_cast<uint8_t>(Command::paset),
                              .memory_write_command = static_cast<uint8_t>(Command::ramwr)}) {}

  bool initialize() override {
    display_drivers::init_pins(config_.reset_pin, config_.data_command_pin, config_.reset_value);
    auto init_cmds = std::to_array<display_drivers::DisplayInitCmd<Command>>({
        {Command::slpout, {}, 120},
        {Command::noron},
        {config_.invert_colors ? Command::invon : Command::invoff},
#ifdef CONFIG_LV_COLOR_DEPTH_16
        {Command::colmod,
         { 0x05 }},
#else
        {Command::colmod, {0x07}},
#endif
        {Command::dispon},
        {Command::wrctrldp, {0x28}},
        {Command::wrdpbr, {0xFF}, 10},
    });
    send_commands(init_cmds);
    return true;
  }

  void set_rotation(const DisplayRotation &rotation) override {
    Controller::set_rotation(rotation);
    auto data = std::array<uint8_t, 1>{make_madctl(rotation)};
    std::scoped_lock lock(io_mutex_);
    write_command(static_cast<uint8_t>(Command::madctl), data, 0);
  }

  void set_brightness(float brightness) {
    brightness_ = std::clamp(brightness, 0.0f, 1.0f);
    uint16_t value = brightness_ * 1023;
    write_command(static_cast<uint8_t>(Command::wrdpbr),
                  {reinterpret_cast<const uint8_t *>(&value), 2}, 0);
  }

  float get_brightness() const { return brightness_; }

private:
  uint8_t make_madctl(DisplayRotation rotation) const {
    auto value = display_drivers::make_madctl_base(config_, LCD_CMD_BGR_BIT, LCD_CMD_MX_BIT,
                                                   LCD_CMD_MY_BIT, LCD_CMD_MV_BIT);
    return display_drivers::apply_standard_rotation(value, config_, rotation, LCD_CMD_MX_BIT,
                                                    LCD_CMD_MY_BIT, LCD_CMD_MV_BIT);
  }

  float brightness_{0.0f};
};
} // namespace espp
