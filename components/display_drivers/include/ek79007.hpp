#pragma once

#include <mutex>

#include "display_drivers.hpp"

namespace espp {
/**
 * @brief Display driver for the EK79007 MIPI-DSI display controller.
 *
 * The EK79007 (e.g. the 7-inch 1024x600 panel on the ESP32-P4-HMI-Subboard) is
 * largely driven by its DPI video timing; it only needs a short vendor command
 * sequence plus a DSI lane-count command and Sleep-Out. This follows the same
 * interface as the other espp display drivers and relies on a lower-level
 * transport (MIPI-DSI DBI) to execute write_command.
 *
 * The initialization sequence here mirrors Espressif's esp_lcd_ek79007 driver.
 */
class Ek79007 : public display_drivers::MipiDbiDisplayDriver {
  // EK79007 MADCTL mirror bits (non-standard layout)
  static constexpr uint8_t SHLR_BIT = 1 << 0; ///< Source (horizontal) flip -> mirror x
  static constexpr uint8_t UPDN_BIT = 1 << 1; ///< Gate (vertical) flip -> mirror y

public:
  enum class Command : uint8_t {
    nop = 0x00,         ///< No Operation
    swreset = 0x01,     ///< Software Reset
    sleep_in = 0x10,    ///< Sleep In
    sleep_out = 0x11,   ///< Sleep Out
    display_off = 0x28, ///< Display Off
    display_on = 0x29,  ///< Display On
    caset = 0x2A,       ///< Column Address Set
    raset = 0x2B,       ///< Row Address Set
    ramwr = 0x2C,       ///< Memory Write
    madctl = 0x36,      ///< Memory Data Access Control
    pad_control = 0xB2, ///< DSI lane configuration
  };

  /// DSI lane configuration values for the pad_control (0xB2) command
  static constexpr uint8_t DSI_2_LANE = 0x10;
  static constexpr uint8_t DSI_4_LANE = 0x00;

  explicit Ek79007(const display_drivers::Config &config)
      : MipiDbiDisplayDriver(config,
                             {.column_address_command = static_cast<uint8_t>(Command::caset),
                              .row_address_command = static_cast<uint8_t>(Command::raset),
                              .memory_write_command = static_cast<uint8_t>(Command::ramwr)}) {}

  bool initialize() override {
    display_drivers::init_pins(config_.reset_pin, config_.data_command_pin, config_.reset_value);

    // This must match Espressif's esp_lcd_ek79007 vendor_specific_init_default
    // exactly: the DSI lane-config command, the 0x80-0x86 vendor registers, then
    // Sleep-Out. Notably it does NOT send MADCTL or Display-On here (the EK79007
    // uses its power-on defaults + the DPI video stream); sending those extra
    // commands can leave the panel showing black.
    auto init_commands = std::to_array<display_drivers::DisplayInitCmd<>>({
        // Configure DSI for 2 data lanes
        {static_cast<uint8_t>(Command::pad_control), {DSI_2_LANE}, 0},
        // Vendor-specific power/timing setup
        {0x80, {0x8B}, 0},
        {0x81, {0x78}, 0},
        {0x82, {0x84}, 0},
        {0x83, {0x88}, 0},
        {0x84, {0xA8}, 0},
        {0x85, {0xE3}, 0},
        {0x86, {0x88}, 0},
        // Exit sleep (requires >=120ms before sending further commands)
        {static_cast<uint8_t>(Command::sleep_out), {}, 120},
    });

    send_commands(init_commands);
    return true;
  }

  void set_rotation(const DisplayRotation &rotation) override {
    Controller::set_rotation(rotation);
    auto data = std::array<uint8_t, 1>{make_madctl()};
    std::scoped_lock lock(io_mutex_);
    write_command(static_cast<uint8_t>(Command::madctl), data, 0);
  }

private:
  uint8_t make_madctl() const {
    uint8_t value = 0;
    if (config_.mirror_x)
      value |= SHLR_BIT;
    if (config_.mirror_y)
      value |= UPDN_BIT;
    return value;
  }
};
} // namespace espp
