#pragma once

#include <mutex>

#include "display_drivers.hpp"

namespace espp {
/**
 * @brief Display driver for the JD9365 MIPI-DSI display controller.
 *
 * The JD9365 (e.g. the Waveshare 10.1" 800x1280 DSI panel used with the
 * ESP32-P4-ETH / ESP32-P4-NANO boards) is configured over the MIPI-DSI DBI
 * (command) channel and then driven by the DPI video stream. This follows the
 * same interface as the other espp display drivers and relies on a lower-level
 * transport to execute write_command.
 *
 * The vendor initialization sequence (the page-selected register writes below)
 * is taken verbatim from the default init table
 * (`vendor_specific_init_default[]`) of Waveshare's esp_lcd_jd9365 v2.0.0
 * managed component (Apache-2.0, Copyright 2024 Espressif Systems (Shanghai)
 * CO LTD), as is the command ordering around it (software reset; user page /
 * MADCTL / COLMOD / DSI lane config; vendor table ending in Sleep-Out +
 * Display-On).
 */
class Jd9365 : public display_drivers::MipiDbiDisplayDriver {
  // JD9365 MADCTL mirror bits (gate scan / source scan direction)
  static constexpr uint8_t GS_BIT = 1 << 0; ///< Gate scan direction -> mirror x
  static constexpr uint8_t SS_BIT = 1 << 1; ///< Source scan direction -> mirror y

public:
  enum class Command : uint8_t {
    nop = 0x00,         ///< No Operation
    swreset = 0x01,     ///< Software Reset
    sleep_in = 0x10,    ///< Sleep In
    sleep_out = 0x11,   ///< Sleep Out
    invert_off = 0x20,  ///< Display Inversion Off
    invert_on = 0x21,   ///< Display Inversion On
    display_off = 0x28, ///< Display Off
    display_on = 0x29,  ///< Display On
    caset = 0x2A,       ///< Column Address Set
    raset = 0x2B,       ///< Row Address Set
    ramwr = 0x2C,       ///< Memory Write
    madctl = 0x36,      ///< Memory Data Access Control
    colmod = 0x3A,      ///< Pixel Format Set
    dsi_int0 = 0x80,    ///< DSI interface config (lane count), on the user page
    page_select = 0xE0, ///< Command page select (0x00 = user page)
  };

  /// Values for the dsi_int0 (0x80) DSI lane-count command
  static constexpr uint8_t DSI_1_LANE = 0x00;
  static constexpr uint8_t DSI_2_LANE = 0x01;
  static constexpr uint8_t DSI_3_LANE = 0x10;
  static constexpr uint8_t DSI_4_LANE = 0x11;

  /// Value for page_select (0xE0) selecting the user command page
  static constexpr uint8_t PAGE_USER = 0x00;

  explicit Jd9365(const display_drivers::Config &config)
      : MipiDbiDisplayDriver(config,
                             {.column_address_command = static_cast<uint8_t>(Command::caset),
                              .row_address_command = static_cast<uint8_t>(Command::raset),
                              .memory_write_command = static_cast<uint8_t>(Command::ramwr)}) {}

  bool initialize() override {
    using namespace std::chrono_literals;
    display_drivers::init_pins(config_.reset_pin, config_.data_command_pin, config_.reset_value);

    // Neither supported board routes a panel reset GPIO, so do what the vendor
    // component's panel_jd9365_reset() does in that case: a DCS software reset
    // followed by a 120 ms delay.
    if (config_.reset_pin == GPIO_NUM_NC) {
      write_command(static_cast<uint8_t>(Command::swreset), {}, 0);
      std::this_thread::sleep_for(120ms);
    }

    auto madctl = make_madctl(DisplayRotation::LANDSCAPE);

    uint8_t colmod = 0x55;
    switch (config_.bits_per_pixel) {
    case 16: // RGB565
      colmod = 0x55;
      break;
    case 18: // RGB666
      colmod = 0x66;
      break;
    case 24: // RGB888
      colmod = 0x77;
      break;
    default:
      break;
    }

    auto init_commands = std::to_array<display_drivers::DisplayInitCmd<>>({
        // Select the user command page, then MADCTL / COLMOD / DSI lane count
        // (2 data lanes), in the same order the vendor component's
        // panel_jd9365_init() sends them before the vendor init table.
        {static_cast<uint8_t>(Command::page_select), {PAGE_USER}, 0},
        {static_cast<uint8_t>(Command::madctl), {madctl}, 0},
        {static_cast<uint8_t>(Command::colmod), {colmod}, 0},
        {static_cast<uint8_t>(Command::dsi_int0), {DSI_2_LANE}, 0},

        // Vendor init table (verbatim from Waveshare's esp_lcd_jd9365 v2.0.0
        // vendor_specific_init_default[], default 800x1280 10.1" panel table).
        {0xE0, {0x00}, 0},
        {0xE1, {0x93}, 0},
        {0xE2, {0x65}, 0},
        {0xE3, {0xF8}, 0},
        {0x80, {0x01}, 0},

        // Select command page 1
        {0xE0, {0x01}, 0},
        {0x00, {0x00}, 0},
        {0x01, {0x38}, 0},
        {0x03, {0x10}, 0},
        {0x04, {0x38}, 0},

        {0x0C, {0x74}, 0},

        {0x17, {0x00}, 0},
        {0x18, {0xAF}, 0},
        {0x19, {0x00}, 0},
        {0x1A, {0x00}, 0},
        {0x1B, {0xAF}, 0},
        {0x1C, {0x00}, 0},

        {0x35, {0x26}, 0},

        {0x37, {0x09}, 0},

        {0x38, {0x04}, 0},
        {0x39, {0x00}, 0},
        {0x3A, {0x01}, 0},
        {0x3C, {0x78}, 0},
        {0x3D, {0xFF}, 0},
        {0x3E, {0xFF}, 0},
        {0x3F, {0x7F}, 0},

        {0x40, {0x06}, 0},
        {0x41, {0xA0}, 0},
        {0x42, {0x81}, 0},
        {0x43, {0x1E}, 0},
        {0x44, {0x0D}, 0},
        {0x45, {0x28}, 0},
        // SKIP: //{0x4A, (uint8_t[]){0x35}, 1, 0},//bist

        {0x55, {0x02}, 0},
        {0x57, {0x69}, 0},
        {0x59, {0x0A}, 0},
        {0x5A, {0x2A}, 0},
        {0x5B, {0x17}, 0},

        {0x5D, {0x7F}, 0},
        {0x5E, {0x6A}, 0},
        {0x5F, {0x5B}, 0},
        {0x60, {0x4F}, 0},
        {0x61, {0x4A}, 0},
        {0x62, {0x3D}, 0},
        {0x63, {0x41}, 0},
        {0x64, {0x2A}, 0},
        {0x65, {0x44}, 0},
        {0x66, {0x43}, 0},
        {0x67, {0x44}, 0},
        {0x68, {0x62}, 0},
        {0x69, {0x52}, 0},
        {0x6A, {0x59}, 0},
        {0x6B, {0x4C}, 0},
        {0x6C, {0x48}, 0},
        {0x6D, {0x3A}, 0},
        {0x6E, {0x26}, 0},
        {0x6F, {0x00}, 0},
        {0x70, {0x7F}, 0},
        {0x71, {0x6A}, 0},
        {0x72, {0x5B}, 0},
        {0x73, {0x4F}, 0},
        {0x74, {0x4A}, 0},
        {0x75, {0x3D}, 0},
        {0x76, {0x41}, 0},
        {0x77, {0x2A}, 0},
        {0x78, {0x44}, 0},
        {0x79, {0x43}, 0},
        {0x7A, {0x44}, 0},
        {0x7B, {0x62}, 0},
        {0x7C, {0x52}, 0},
        {0x7D, {0x59}, 0},
        {0x7E, {0x4C}, 0},
        {0x7F, {0x48}, 0},
        {0x80, {0x3A}, 0},
        {0x81, {0x26}, 0},
        {0x82, {0x00}, 0},

        // Select command page 2
        {0xE0, {0x02}, 0},
        {0x00, {0x42}, 0},
        {0x01, {0x42}, 0},
        {0x02, {0x40}, 0},
        {0x03, {0x40}, 0},
        {0x04, {0x5E}, 0},
        {0x05, {0x5E}, 0},
        {0x06, {0x5F}, 0},
        {0x07, {0x5F}, 0},
        {0x08, {0x5F}, 0},
        {0x09, {0x57}, 0},
        {0x0A, {0x57}, 0},
        {0x0B, {0x77}, 0},
        {0x0C, {0x77}, 0},
        {0x0D, {0x47}, 0},
        {0x0E, {0x47}, 0},
        {0x0F, {0x45}, 0},
        {0x10, {0x45}, 0},
        {0x11, {0x4B}, 0},
        {0x12, {0x4B}, 0},
        {0x13, {0x49}, 0},
        {0x14, {0x49}, 0},
        {0x15, {0x5F}, 0},

        {0x16, {0x41}, 0},
        {0x17, {0x41}, 0},
        {0x18, {0x40}, 0},
        {0x19, {0x40}, 0},
        {0x1A, {0x5E}, 0},
        {0x1B, {0x5E}, 0},
        {0x1C, {0x5F}, 0},
        {0x1D, {0x5F}, 0},
        {0x1E, {0x5F}, 0},
        {0x1F, {0x57}, 0},
        {0x20, {0x57}, 0},
        {0x21, {0x77}, 0},
        {0x22, {0x77}, 0},
        {0x23, {0x46}, 0},
        {0x24, {0x46}, 0},
        {0x25, {0x44}, 0},
        {0x26, {0x44}, 0},
        {0x27, {0x4A}, 0},
        {0x28, {0x4A}, 0},
        {0x29, {0x48}, 0},
        {0x2A, {0x48}, 0},
        {0x2B, {0x5F}, 0},

        {0x2C, {0x01}, 0},
        {0x2D, {0x01}, 0},
        {0x2E, {0x00}, 0},
        {0x2F, {0x00}, 0},
        {0x30, {0x1F}, 0},
        {0x31, {0x1F}, 0},
        {0x32, {0x1E}, 0},
        {0x33, {0x1E}, 0},
        {0x34, {0x1F}, 0},
        {0x35, {0x17}, 0},
        {0x36, {0x17}, 0},
        {0x37, {0x37}, 0},
        {0x38, {0x37}, 0},
        {0x39, {0x08}, 0},
        {0x3A, {0x08}, 0},
        {0x3B, {0x0A}, 0},
        {0x3C, {0x0A}, 0},
        {0x3D, {0x04}, 0},
        {0x3E, {0x04}, 0},
        {0x3F, {0x06}, 0},
        {0x40, {0x06}, 0},
        {0x41, {0x1F}, 0},

        {0x42, {0x02}, 0},
        {0x43, {0x02}, 0},
        {0x44, {0x00}, 0},
        {0x45, {0x00}, 0},
        {0x46, {0x1F}, 0},
        {0x47, {0x1F}, 0},
        {0x48, {0x1E}, 0},
        {0x49, {0x1E}, 0},
        {0x4A, {0x1F}, 0},
        {0x4B, {0x17}, 0},
        {0x4C, {0x17}, 0},
        {0x4D, {0x37}, 0},
        {0x4E, {0x37}, 0},
        {0x4F, {0x09}, 0},
        {0x50, {0x09}, 0},
        {0x51, {0x0B}, 0},
        {0x52, {0x0B}, 0},
        {0x53, {0x05}, 0},
        {0x54, {0x05}, 0},
        {0x55, {0x07}, 0},
        {0x56, {0x07}, 0},
        {0x57, {0x1F}, 0},

        {0x58, {0x40}, 0},
        {0x5B, {0x30}, 0},
        {0x5C, {0x00}, 0},
        {0x5D, {0x34}, 0},
        {0x5E, {0x05}, 0},
        {0x5F, {0x02}, 0},
        {0x63, {0x00}, 0},
        {0x64, {0x6A}, 0},
        {0x67, {0x73}, 0},
        {0x68, {0x07}, 0},
        {0x69, {0x08}, 0},
        {0x6A, {0x6A}, 0},
        {0x6B, {0x08}, 0},

        {0x6C, {0x00}, 0},
        {0x6D, {0x00}, 0},
        {0x6E, {0x00}, 0},
        {0x6F, {0x88}, 0},

        {0x75, {0xFF}, 0},
        {0x77, {0xDD}, 0},
        {0x78, {0x2C}, 0},
        {0x79, {0x15}, 0},
        {0x7A, {0x17}, 0},
        {0x7D, {0x14}, 0},
        {0x7E, {0x82}, 0},

        // Select command page 4
        {0xE0, {0x04}, 0},
        {0x00, {0x0E}, 0},
        {0x02, {0xB3}, 0},
        {0x09, {0x61}, 0},
        {0x0E, {0x48}, 0},
        {0x37, {0x58}, 0},
        {0x2B, {0x0F}, 0},

        // Select command page 0 (user page)
        {0xE0, {0x00}, 0},

        {0xE6, {0x02}, 0},
        {0xE7, {0x0C}, 0},

        {static_cast<uint8_t>(Command::sleep_out), {0x00}, 120}, // Sleep-Out (+120 ms)

        {static_cast<uint8_t>(Command::display_on), {0x00}, 20}, // Display-On (+20 ms)
    });

    send_commands(init_commands);
    return true;
  }

  void set_rotation(const DisplayRotation &rotation) override {
    Controller::set_rotation(rotation);
    auto data = std::array<uint8_t, 1>{make_madctl(rotation)};
    auto page_user = std::array<uint8_t, 1>{PAGE_USER};
    std::scoped_lock lock(io_mutex_);
    write_command(static_cast<uint8_t>(Command::page_select), page_user, 0);
    write_command(static_cast<uint8_t>(Command::madctl), data, 0);
  }

private:
  uint8_t make_madctl(DisplayRotation rotation) const {
    auto value = display_drivers::make_madctl_base(config_, LCD_CMD_BGR_BIT, GS_BIT, SS_BIT, 0);
    return display_drivers::apply_standard_rotation(value, config_, rotation, GS_BIT, SS_BIT, 0);
  }
};
} // namespace espp
