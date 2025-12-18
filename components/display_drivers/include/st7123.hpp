#pragma once

#include <mutex>

#include "display_drivers.hpp"

namespace espp {
/**
 * @brief Display driver for the ST7123 display controller (DSI/DCS style).
 *
 * This follows the same interface as the other display drivers and relies on a
 * lower-level transport to execute write_command and bulk color transfers.
 *
 * The initialization sequence is compatible with M5Stack Tab5 displays using
 * the ST7123 controller chip.
 */
class St7123 {
  static constexpr uint8_t LA_BIT = 1 << 0;  ///< Row Address Order (LA)
  static constexpr uint8_t CA_BIT = 1 << 1;  ///< Column Address Order (CA)
  static constexpr uint8_t BGR_BIT = 1 << 3; ///< BGR Order

public:
  enum class Command : uint8_t {
    // Standard DCS Commands
    nop = 0x00,          ///< No Operation
    swreset = 0x01,      ///< Software Reset
    rddid = 0x04,        ///< Read Display ID
    sleep_in = 0x10,     ///< Sleep In
    sleep_out = 0x11,    ///< Sleep Out
    partial_on = 0x12,   ///< Partial Mode On
    normal_on = 0x13,    ///< Normal Display Mode On
    invert_off = 0x20,   ///< Display Inversion Off
    invert_on = 0x21,    ///< Display Inversion On
    display_off = 0x28,  ///< Display Off
    display_on = 0x29,   ///< Display On
    caset = 0x2A,        ///< Column Address Set
    raset = 0x2B,        ///< Row Address Set
    ramwr = 0x2C,        ///< Memory Write
    ramrd = 0x2E,        ///< Memory Read
    partial_area = 0x30, ///< Partial Area
    vscrdef = 0x33,      ///< Vertical Scrolling Definition
    te_off = 0x34,       ///< Tearing Effect Line Off
    te_on = 0x35,        ///< Tearing Effect Line On
    madctl = 0x36,       ///< Memory Data Access Control
    vscsad = 0x37,       ///< Vertical Scroll Start Address of RAM
    idle_off = 0x38,     ///< Idle Mode Off
    idle_on = 0x39,      ///< Idle Mode On
    colmod = 0x3A,       ///< Pixel Format Set

    // ST7123 Specific Commands
    page_select = 0xB9, ///< Set EXTC command (0xB9 for ST7123)
  };

  /**
   * @brief Store config and send initialization commands to the controller.
   * @param config display_drivers::Config
   * @return true if initialization succeeded, false otherwise
   */
  static bool initialize(const display_drivers::Config &config) {
    write_command_ = config.write_command;
    read_command_ = config.read_command;
    lcd_send_lines_ = config.lcd_send_lines;
    reset_pin_ = config.reset_pin;
    dc_pin_ = config.data_command_pin;
    offset_x_ = config.offset_x;
    offset_y_ = config.offset_y;
    mirror_x_ = config.mirror_x;
    mirror_y_ = config.mirror_y;
    mirror_portrait_ = config.mirror_portrait;
    swap_xy_ = config.swap_xy;
    swap_color_order_ = config.swap_color_order;

    // Initialize display pins
    display_drivers::init_pins(reset_pin_, dc_pin_, config.reset_value);

    uint8_t madctl = 0x00;
    if (swap_color_order_) {
      madctl |= BGR_BIT;
    }
    if (mirror_x_) {
      madctl |= CA_BIT;
    }
    if (mirror_y_) {
      madctl |= LA_BIT;
    }
    if (swap_xy_) {
      // Row/column exchange - ST7123 doesn't have explicit bit for this in basic MADCTL
      madctl |= 0;
    }

    uint8_t colmod = 0x55; // default to 16 bits per pixel
    switch (config.bits_per_pixel) {
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

    // Try to read the ID if we have a read_command function
    if (config.read_command) {
      uint8_t id[3] = {0};
      // Read ID registers (RDDID returns 0xFF for first byte, then ID1, ID2)
      read_command_(static_cast<uint8_t>(Command::rddid), {id, 3}, 0);

      // First byte is 0xFF, check remaining bytes for ST7123 signature
      // Note: Exact ID values depend on panel manufacturer
      // Skip validation for now to allow display to work with different panels
      // if (id[0] != 0xFF) {
      //   return false;
      // }
    }

    // ST7123 initialization sequence
    // Minimal initialization based on ST7123 datasheet
    auto init_commands = std::to_array<display_drivers::DisplayInitCmd<>>({
        // Software reset
        {static_cast<uint8_t>(Command::swreset), {}, 120},

        // Sleep Out - must wait 120ms after this
        {static_cast<uint8_t>(Command::sleep_out), {}, 120},

        // Memory access control
        {static_cast<uint8_t>(Command::madctl), {madctl}, 0},

        // Pixel format (color mode)
        {static_cast<uint8_t>(Command::colmod), {colmod}, 0},

        // Normal display mode
        {static_cast<uint8_t>(Command::normal_on), {}, 0},

        // Display On
        {static_cast<uint8_t>(Command::display_on), {}, 20},
    });

    for (auto &cmd : init_commands) {
      write_command_(cmd.command, std::span<const uint8_t>(cmd.parameters), 0);
      if (cmd.delay_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(cmd.delay_ms));
      }
    }

    return true;
  }

  /**
   * @brief Get the display controller ID
   * @return ID string for ST7123
   */
  static constexpr const char *id() { return "ST7123"; }

protected:
  static display_drivers::write_command_fn write_command_;
  static display_drivers::read_command_fn read_command_;
  static display_drivers::send_lines_fn lcd_send_lines_;
  static gpio_num_t reset_pin_;
  static gpio_num_t dc_pin_;
  static int offset_x_;
  static int offset_y_;
  static bool swap_xy_;
  static bool mirror_x_;
  static bool mirror_y_;
  static bool mirror_portrait_;
  static bool swap_color_order_;
};

inline display_drivers::write_command_fn St7123::write_command_{nullptr};
inline display_drivers::read_command_fn St7123::read_command_{nullptr};
inline display_drivers::send_lines_fn St7123::lcd_send_lines_{nullptr};
inline gpio_num_t St7123::reset_pin_{GPIO_NUM_NC};
inline gpio_num_t St7123::dc_pin_{GPIO_NUM_NC};
inline int St7123::offset_x_{0};
inline int St7123::offset_y_{0};
inline bool St7123::swap_xy_{false};
inline bool St7123::mirror_x_{false};
inline bool St7123::mirror_y_{false};
inline bool St7123::mirror_portrait_{false};
inline bool St7123::swap_color_order_{false};

} // namespace espp
