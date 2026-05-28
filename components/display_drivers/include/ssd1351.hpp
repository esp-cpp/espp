#pragma once

#include "display_drivers.hpp"

namespace espp {
/**
 * @brief Display driver for the SSD1351 128x128 RGB OLED display controller.
 *
 * \section ssd1351_byte90_cfg Byte90 Ssd1351 Config
 * \snippet display_drivers_example.cpp byte90_config example
 * \section ssd1351_ex1 ssd1351 Example
 * \snippet display_drivers_example.cpp display_drivers example
 */
class Ssd1351 : public display_drivers::MipiDbiDisplayDriver {
public:
  enum class Command : uint8_t {
    caset = 0x15,
    raset = 0x75,
    ramwr = 0x5C,
    ramrd = 0x5D,
    madctl = 0xA0,
    STARTLINE = 0xA1,
    DISPLAYOFFSET = 0xA2,
    DISPLAYALLOFF = 0xA4,
    DISPLAYALLON = 0xA5,
    NORMALDISPLAY = 0xA6,
    INVERTDISPLAY = 0xA7,
    FUNCTIONSELECT = 0xAB,
    DISPLAYOFF = 0xAE,
    DISPLAYON = 0xAF,
    PRECHARGE = 0xB1,
    DISPLAYENHANCE = 0xB2,
    CLOCKDIV = 0xB3,
    SETVSL = 0xB4,
    SETGPIO = 0xB5,
    PRECHARGE2 = 0xB6,
    SETGRAY = 0xB8,
    USELUT = 0xB9,
    PRECHARGELEVEL = 0xBB,
    VCOMH = 0xBE,
    CONTRASTABC = 0xC1,
    CONTRASTMASTER = 0xC7,
    MUXRATIO = 0xCA,
    HORIZSCROLL = 0x96,
    STOPSCROLL = 0x9E,
    STARTSCROLL = 0x9F,
    nop0 = 0xD1,
    nop1 = 0xE3,
    COMMANDLOCK = 0xFD,
  };

  static constexpr int OLED_CMD_COLOR_DEPTH_65K2 = 0b01 << 6;
  static constexpr int OLED_CMD_COM_SPLIT = 0b1 << 5;
  static constexpr int OLED_CMD_MY_BIT = 0b1 << 4;
  static constexpr int OLED_CMD_BGR_BIT = 0b1 << 2;
  static constexpr int OLED_CMD_MX_BIT = 0b1 << 1;
  static constexpr int OLED_CMD_MV_BIT = 0b1 << 0;
  static constexpr int DEFAULT_MADCTL = OLED_CMD_COLOR_DEPTH_65K2 | OLED_CMD_COM_SPLIT;

  explicit Ssd1351(const display_drivers::Config &config)
      : MipiDbiDisplayDriver(config,
                             {.column_address_command = static_cast<uint8_t>(Command::caset),
                              .row_address_command = static_cast<uint8_t>(Command::raset),
                              .memory_write_command = static_cast<uint8_t>(Command::ramwr),
                              .use_8bit_coordinates = true}) {}

  bool initialize() override {
    display_drivers::init_pins(config_.reset_pin, config_.data_command_pin, config_.reset_value);

    auto madctl = make_madctl(DisplayRotation::LANDSCAPE);
    auto init_commands = std::to_array<display_drivers::DisplayInitCmd<>>({
        {static_cast<uint8_t>(Command::COMMANDLOCK), {0x12}, 0},
        {static_cast<uint8_t>(Command::COMMANDLOCK), {0xB1}, 0},
        {static_cast<uint8_t>(Command::DISPLAYOFF), {}, 0},
        {static_cast<uint8_t>(Command::CLOCKDIV), {0xF1}, 0},
        {static_cast<uint8_t>(Command::MUXRATIO), {0x7F}, 0},
        {static_cast<uint8_t>(Command::DISPLAYOFFSET), {0x00}, 0},
        {static_cast<uint8_t>(Command::SETGPIO), {0x00}, 0},
        {static_cast<uint8_t>(Command::FUNCTIONSELECT), {0x01}, 0},
        {static_cast<uint8_t>(Command::PRECHARGE), {0x32}, 0},
        {static_cast<uint8_t>(Command::VCOMH), {0x05}, 0},
        {config_.invert_colors ? static_cast<uint8_t>(Command::INVERTDISPLAY)
                               : static_cast<uint8_t>(Command::NORMALDISPLAY),
         {},
         0},
        {static_cast<uint8_t>(Command::CONTRASTABC), {0xC8, 0x80, 0xC8}, 0},
        {static_cast<uint8_t>(Command::CONTRASTMASTER), {0x0F}, 0},
        {static_cast<uint8_t>(Command::SETVSL), {0xA0, 0xB5, 0x55}, 0},
        {static_cast<uint8_t>(Command::PRECHARGE2), {0x01}, 0},
        {static_cast<uint8_t>(Command::PRECHARGELEVEL), {0x1C, 0x1C, 0x1C}, 0},
        {static_cast<uint8_t>(Command::DISPLAYON), {}, 0},
        {static_cast<uint8_t>(Command::madctl), {madctl}, 0},
        {static_cast<uint8_t>(Command::STARTLINE), {0x00}, 0},
    });

    send_commands(init_commands);
    return true;
  }

  void set_rotation(const DisplayRotation &rotation) override {
    Controller::set_rotation(rotation);

    auto madctl = make_madctl(rotation);
    uint8_t startline = 0;
    switch (rotation) {
    case DisplayRotation::LANDSCAPE:
    case DisplayRotation::PORTRAIT:
      startline = 127;
      break;
    case DisplayRotation::LANDSCAPE_INVERTED:
    case DisplayRotation::PORTRAIT_INVERTED:
      startline = 0;
      break;
    }

    auto startline_data = std::array<uint8_t, 1>{startline};
    auto madctl_data = std::array<uint8_t, 1>{madctl};
    std::scoped_lock lock(io_mutex_);
    write_command(static_cast<uint8_t>(Command::STARTLINE), startline_data, 0);
    write_command(static_cast<uint8_t>(Command::madctl), madctl_data, 0);
  }

  void set_brightness(float brightness) {
    brightness_ = std::clamp(brightness, 0.0f, 1.0f);
    auto data = std::array<uint8_t, 1>{static_cast<uint8_t>(brightness_ * 15.0f)};
    std::scoped_lock lock(io_mutex_);
    write_command(static_cast<uint8_t>(Command::CONTRASTMASTER), data, 0);
  }

  float get_brightness() const { return brightness_; }

protected:
  display_drivers::Region transform_region(display_drivers::Region region) const override {
    switch (rotation_) {
    case DisplayRotation::PORTRAIT:
    case DisplayRotation::PORTRAIT_INVERTED:
      std::swap(region.xs, region.ys);
      std::swap(region.xe, region.ye);
      break;
    case DisplayRotation::LANDSCAPE:
    case DisplayRotation::LANDSCAPE_INVERTED:
    default:
      break;
    }
    return region;
  }

private:
  uint8_t make_madctl(DisplayRotation rotation) const {
    auto value = DEFAULT_MADCTL;
    if (config_.swap_color_order) {
      value |= OLED_CMD_BGR_BIT;
    }
    if (config_.mirror_x) {
      value |= OLED_CMD_MX_BIT;
    }
    if (config_.mirror_y) {
      value |= OLED_CMD_MY_BIT;
    }
    if (config_.swap_xy) {
      value |= OLED_CMD_MV_BIT;
    }

    switch (rotation) {
    case DisplayRotation::LANDSCAPE:
      break;
    case DisplayRotation::PORTRAIT:
      value ^= OLED_CMD_MV_BIT;
      value ^= config_.mirror_portrait ? OLED_CMD_MY_BIT : OLED_CMD_MX_BIT;
      break;
    case DisplayRotation::LANDSCAPE_INVERTED:
      value ^= (OLED_CMD_MY_BIT | OLED_CMD_MX_BIT);
      break;
    case DisplayRotation::PORTRAIT_INVERTED:
      value ^= OLED_CMD_MV_BIT;
      value ^= config_.mirror_portrait ? OLED_CMD_MX_BIT : OLED_CMD_MY_BIT;
      break;
    }
    return value;
  }

  float brightness_{1.0f};
};
} // namespace espp
