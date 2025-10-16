#pragma once

#include <mutex>

#include "display_drivers.hpp"

namespace espp {
/**
 * @brief Display driver for the ILI9881C display controller (DSI/DCS style).
 *
 * This follows the same interface as the other display drivers and relies on a
 * lower-level transport to execute write_command and bulk color transfers.
 *
 * The initialization sequence includes the comprehensive M5Stack Tab5 setup
 * with GIP (Gate In Panel) timing control, power management, and gamma correction
 * for optimal display quality.
 */
class Ili9881 {
  static constexpr uint8_t GS_BIT = 1 << 0;
  static constexpr uint8_t SS_BIT = 1 << 1;

public:
  enum class Command : uint8_t {
    // Standard DCS Commands
    nop = 0x00,          ///< No Operation
    swreset = 0x01,      ///< Software Reset
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

    // ILI9881C Specific Commands
    page_select = 0xFF, ///< Page Select Command (0xFF)

    // Command Page 1 Registers
    mipi_ctrl = 0x22,  ///< MIPI Control (Page 1)
    inv_ctrl1 = 0x31,  ///< Inversion Control 1 (Page 1)
    vreg_ctrl1 = 0x50, ///< VREG Control 1 (Page 1)
    vreg_ctrl2 = 0x51, ///< VREG Control 2 (Page 1)
    vreg_ctrl3 = 0x53, ///< VREG Control 3 (Page 1)
    vreg_ctrl4 = 0x55, ///< VREG Control 4 (Page 1)
    bias_ctrl = 0x60,  ///< Bias Current Control (Page 1)
    bias_ctrl2 = 0x61, ///< Bias Control 2 (Page 1)
    bias_ctrl3 = 0x62, ///< Bias Control 3 (Page 1)
    bias_ctrl4 = 0x63, ///< Bias Control 4 (Page 1)
    dsi_ctrl = 0xB7,   ///< DSI Control (Page 1)

    // Gamma Correction Commands (Page 1)
    gmctr_p1 = 0xA0,  ///< Positive Gamma Control 1
    gmctr_p20 = 0xB3, ///< Positive Gamma Control 20
    gmctr_n1 = 0xC0,  ///< Negative Gamma Control 1
    gmctr_n20 = 0xD3, ///< Negative Gamma Control 20

    // Command Page 3 - GIP (Gate In Panel) Controls
    gip_1 = 0x01,     ///< Gate Driver Control 1 (Page 3)
    gip_68 = 0x44,    ///< Gate Driver Control 68 (Page 3)
    gip_r_l1 = 0x50,  ///< Forward Scan Signal Output 1 (Page 3)
    gip_r_l14 = 0x5D, ///< Forward Scan Signal Output 14 (Page 3)
    gip_l_l1 = 0x5E,  ///< Backward Scan Signal Output 1 (Page 3)
    gip_eq = 0x8A,    ///< Gate Equalization Control (Page 3)

    // Command Page 4 - Power Control
    vreg1out = 0x39,     ///< VREG1 Output Control (Page 4)
    vreg2out = 0x38,     ///< VREG2 Output Enable (Page 4)
    power_ctrl = 0x3A,   ///< Power Control Setting (Page 4)
    vgh_vgl_ctrl = 0x3B, ///< VGH/VGL Timing Control (Page 4)
    vgh_clamp = 0x6C,    ///< VGH Clamp Voltage Setting (Page 4)
    vgl_clamp = 0x6E,    ///< VGL Clamp Voltage Setting (Page 4)
    pump_clamp = 0x6F,   ///< Charge Pump Clamp Setting (Page 4)
    vcore_volt = 0x87,   ///< VCORE Voltage Setting (Page 4)
    vcl_volt = 0x8D,     ///< VCL Voltage Level Setting (Page 4)

    // Extended/Custom Commands
    nop_extended = 0xFE, ///< Extended NOP Command
  };

  /**
   * @brief Store config and send initialization commands to the controller.
   * @param config display_drivers::Config
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
      madctl |= LCD_CMD_BGR_BIT;
    }
    if (mirror_x_) {
      madctl |= GS_BIT; // LCD_CMD_MX_BIT;
    }
    if (mirror_y_) {
      madctl |= SS_BIT; // LCD_CMD_MY_BIT;
    }
    if (swap_xy_) {
      madctl |= 0; // LCD_CMD_MV_BIT;
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

    // first let's read the ID if we have a read_command function
    if (config.read_command) {
      uint8_t id[3] = {0};
      // select cmd page 1
      write_command_(static_cast<uint8_t>(Command::page_select),
                     std::span<const uint8_t>{{0x98, 0x81, 0x01}}, 0);
      // read ID registers
      read_command_(static_cast<uint8_t>(0x00), {&id[0], 1}, 0); // ID1
      read_command_(static_cast<uint8_t>(0x01), {&id[1], 1}, 0); // ID2
      read_command_(static_cast<uint8_t>(0x02), {&id[2], 1}, 0); // ID3

      if (id[0] != 0x98 || id[1] != 0x81 || id[2] != 0x5C) {
        return false;
      }
    }

    // Comprehensive ILI9881C initialization sequence (M5Stack Tab5 specific)
    auto init_commands = std::to_array<display_drivers::DisplayInitCmd<>>({
        // CMD_Page 1 - DSI and Basic Setup
        {static_cast<uint8_t>(Command::page_select),
         {0x98, 0x81, 0x01},
         0}, // Switch to Command Page 1
        // TODO: should we read the ID here?
        {static_cast<uint8_t>(Command::dsi_ctrl), {0x03}, 0}, // Set 2-lane DSI mode

        // CMD_Page 0 - Exit Sleep
        {static_cast<uint8_t>(Command::page_select), {0x98, 0x81, 0x00}, 0}, // Switch to Page 0
        {static_cast<uint8_t>(Command::sleep_out), {}, 120},                 // Sleep Out
        {static_cast<uint8_t>(Command::madctl), {madctl}, 0}, // Memory access control
        {static_cast<uint8_t>(Command::colmod), {colmod}, 0}, // Color mode 16-bit

        // CMD_Page 3 - GIP (Gate In Panel) Configuration
        {static_cast<uint8_t>(Command::page_select),
         {0x98, 0x81, 0x03},
         0},                                               // Switch to Command Page 3
        {static_cast<uint8_t>(Command::gip_1), {0x00}, 0}, // Gate driver control 1
        {0x02, {0x00}, 0},                                 // Gate driver control 2
        {0x03, {0x73}, 0},                                 // Gate driver control 3
        {0x04, {0x00}, 0},
        {0x05, {0x00}, 0},
        {0x06, {0x08}, 0},
        {0x07, {0x00}, 0},
        {0x08, {0x00}, 0},
        {0x09, {0x1B}, 0},
        {0x0a, {0x01}, 0},
        {0x0b, {0x01}, 0},
        {0x0c, {0x0D}, 0},
        {0x0d, {0x01}, 0},
        {0x0e, {0x01}, 0},
        {0x0f, {0x26}, 0},
        {0x10, {0x26}, 0},
        {0x11, {0x00}, 0},
        {0x12, {0x00}, 0},
        {0x13, {0x02}, 0},
        {0x14, {0x00}, 0},
        {0x15, {0x00}, 0},
        {0x16, {0x00}, 0},
        {0x17, {0x00}, 0},
        {0x18, {0x00}, 0},
        {0x19, {0x00}, 0},
        {0x1a, {0x00}, 0},
        {0x1b, {0x00}, 0},
        {0x1c, {0x00}, 0},
        {0x1d, {0x00}, 0},
        {0x1e, {0x40}, 0},
        {0x1f, {0x00}, 0},
        {0x20, {0x06}, 0},
        {0x21, {0x01}, 0},
        {0x22, {0x00}, 0},
        {0x23, {0x00}, 0},
        {0x24, {0x00}, 0},
        {0x25, {0x00}, 0},
        {0x26, {0x00}, 0},
        {0x27, {0x00}, 0},
        {0x28, {0x33}, 0},
        {0x29, {0x03}, 0},
        {0x2a, {0x00}, 0},
        {0x2b, {0x00}, 0},
        {0x2c, {0x00}, 0},
        {0x2d, {0x00}, 0},
        {0x2e, {0x00}, 0},
        {0x2f, {0x00}, 0},
        {0x30, {0x00}, 0},
        {0x31, {0x00}, 0},
        {0x32, {0x00}, 0},
        {0x33, {0x00}, 0},
        {0x34, {0x00}, 0},
        {0x35, {0x00}, 0},
        {0x36, {0x00}, 0},
        {0x37, {0x00}, 0},
        {0x38, {0x00}, 0},
        {0x39, {0x00}, 0},
        {0x3a, {0x00}, 0},
        {0x3b, {0x00}, 0},
        {0x3c, {0x00}, 0},
        {0x3d, {0x00}, 0},
        {0x3e, {0x00}, 0},
        {0x3f, {0x00}, 0},
        {0x40, {0x00}, 0},
        {0x41, {0x00}, 0},
        {0x42, {0x00}, 0},
        {0x43, {0x00}, 0},
        {static_cast<uint8_t>(Command::gip_68), {0x00}, 0}, // Gate driver control 68

        // Forward scan signal outputs
        {static_cast<uint8_t>(Command::gip_r_l1), {0x01}, 0}, // Forward scan 1
        {0x51, {0x23}, 0},
        {0x52, {0x45}, 0},
        {0x53, {0x67}, 0},
        {0x54, {0x89}, 0},
        {0x55, {0xab}, 0},
        {0x56, {0x01}, 0},
        {0x57, {0x23}, 0},
        {0x58, {0x45}, 0},
        {0x59, {0x67}, 0},
        {0x5a, {0x89}, 0},
        {0x5b, {0xab}, 0},
        {0x5c, {0xcd}, 0},
        {static_cast<uint8_t>(Command::gip_r_l14), {0xef}, 0}, // Forward scan 14

        // Backward scan signal outputs
        {static_cast<uint8_t>(Command::gip_l_l1), {0x11}, 0}, // Backward scan 1
        {0x5f, {0x02}, 0},
        {0x60, {0x00}, 0},
        {0x61, {0x07}, 0},
        {0x62, {0x06}, 0},
        {0x63, {0x0E}, 0},
        {0x64, {0x0F}, 0},
        {0x65, {0x0C}, 0},
        {0x66, {0x0D}, 0},
        {0x67, {0x02}, 0},
        {0x68, {0x02}, 0},
        {0x69, {0x02}, 0},
        {0x6a, {0x02}, 0},
        {0x6b, {0x02}, 0},
        {0x6c, {0x02}, 0},
        {0x6d, {0x02}, 0},
        {0x6e, {0x02}, 0},
        {0x6f, {0x02}, 0},
        {0x70, {0x02}, 0},
        {0x71, {0x02}, 0},
        {0x72, {0x02}, 0},
        {0x73, {0x05}, 0}, // Backward scan 22

        // Right side signal outputs
        {0x74, {0x01}, 0},
        {0x75, {0x02}, 0},
        {0x76, {0x00}, 0},
        {0x77, {0x07}, 0},
        {0x78, {0x06}, 0},
        {0x79, {0x0E}, 0},
        {0x7a, {0x0F}, 0},
        {0x7b, {0x0C}, 0},
        {0x7c, {0x0D}, 0},
        {0x7d, {0x02}, 0},
        {0x7e, {0x02}, 0},
        {0x7f, {0x02}, 0},
        {0x80, {0x02}, 0},
        {0x81, {0x02}, 0},
        {0x82, {0x02}, 0},
        {0x83, {0x02}, 0},
        {0x84, {0x02}, 0},
        {0x85, {0x02}, 0},
        {0x86, {0x02}, 0},
        {0x87, {0x02}, 0},
        {0x88, {0x02}, 0},
        {0x89, {0x05}, 0},                                  // Right side 22
        {static_cast<uint8_t>(Command::gip_eq), {0x01}, 0}, // Gate equalization control

        // CMD_Page 4 - Power Control
        {static_cast<uint8_t>(Command::page_select),
         {0x98, 0x81, 0x04},
         0},                                                      // Switch to Command Page 4
        {static_cast<uint8_t>(Command::vreg2out), {0x01}, 0},     // VREG2 output enable
        {static_cast<uint8_t>(Command::vreg1out), {0x00}, 0},     // VREG1 output control
        {static_cast<uint8_t>(Command::vgh_clamp), {0x15}, 0},    // VGH clamp voltage
        {static_cast<uint8_t>(Command::vgl_clamp), {0x1A}, 0},    // VGL clamp voltage
        {static_cast<uint8_t>(Command::pump_clamp), {0x25}, 0},   // Charge pump clamp
        {static_cast<uint8_t>(Command::power_ctrl), {0xA4}, 0},   // Power control setting
        {static_cast<uint8_t>(Command::vcl_volt), {0x20}, 0},     // VCL voltage level
        {static_cast<uint8_t>(Command::vcore_volt), {0xBA}, 0},   // VCORE voltage setting
        {static_cast<uint8_t>(Command::vgh_vgl_ctrl), {0x98}, 0}, // VGH/VGL timing control

        // CMD_Page 1 - VREG, Bias, and Gamma Settings
        {static_cast<uint8_t>(Command::page_select),
         {0x98, 0x81, 0x01},
         0},                                                    // Switch to Command Page 1
        {static_cast<uint8_t>(Command::mipi_ctrl), {0x0A}, 0},  // MIPI interface control
        {static_cast<uint8_t>(Command::inv_ctrl1), {0x00}, 0},  // Inversion control 1
        {static_cast<uint8_t>(Command::vreg_ctrl1), {0x6B}, 0}, // VREG control 1
        {static_cast<uint8_t>(Command::vreg_ctrl2), {0x66}, 0}, // VREG control 2
        {static_cast<uint8_t>(Command::vreg_ctrl3), {0x73}, 0}, // VREG control 3
        {static_cast<uint8_t>(Command::vreg_ctrl4), {0x8B}, 0}, // VREG control 4
        {static_cast<uint8_t>(Command::bias_ctrl), {0x1B}, 0},  // Bias current control
        {static_cast<uint8_t>(Command::bias_ctrl2), {0x01}, 0}, // Bias control 2
        {static_cast<uint8_t>(Command::bias_ctrl3), {0x0C}, 0}, // Bias control 3
        {static_cast<uint8_t>(Command::bias_ctrl4), {0x00}, 0}, // Bias control 4

        // Positive Gamma Correction (20 points)
        {static_cast<uint8_t>(Command::gmctr_p1), {0x00}, 0}, // Positive gamma 1
        {0xA1, {0x15}, 0},
        {0xA2, {0x1F}, 0},
        {0xA3, {0x13}, 0},
        {0xA4, {0x11}, 0},
        {0xA5, {0x21}, 0},
        {0xA6, {0x17}, 0},
        {0xA7, {0x1B}, 0},
        {0xA8, {0x6B}, 0},
        {0xA9, {0x1E}, 0},
        {0xAA, {0x2B}, 0},
        {0xAB, {0x5D}, 0},
        {0xAC, {0x19}, 0},
        {0xAD, {0x14}, 0},
        {0xAE, {0x4B}, 0},
        {0xAF, {0x1D}, 0},
        {0xB0, {0x27}, 0},
        {0xB1, {0x49}, 0},
        {0xB2, {0x5D}, 0},
        {static_cast<uint8_t>(Command::gmctr_p20), {0x39}, 0},

        // Negative Gamma Correction (20 points)
        {static_cast<uint8_t>(Command::gmctr_n1), {0x00}, 0}, // Negative gamma 1
        {0xC1, {0x01}, 0},
        {0xC2, {0x0C}, 0},
        {0xC3, {0x11}, 0},
        {0xC4, {0x15}, 0},
        {0xC5, {0x28}, 0},
        {0xC6, {0x1B}, 0},
        {0xC7, {0x1C}, 0},
        {0xC8, {0x62}, 0},
        {0xC9, {0x1C}, 0},
        {0xCA, {0x29}, 0},
        {0xCB, {0x60}, 0},
        {0xCC, {0x16}, 0},
        {0xCD, {0x17}, 0},
        {0xCE, {0x4A}, 0},
        {0xCF, {0x23}, 0},
        {0xD0, {0x24}, 0},
        {0xD1, {0x4F}, 0},
        {0xD2, {0x5F}, 0},
        {static_cast<uint8_t>(Command::gmctr_n20), {0x39}, 0},

        // CMD_Page 0 - User Commands
        {static_cast<uint8_t>(Command::page_select),
         {0x98, 0x81, 0x00},
         0},                                                  // Switch to Command Page 0
        {static_cast<uint8_t>(Command::te_on), {}, 0},        // Tearing Effect Line ON
        {static_cast<uint8_t>(Command::nop_extended), {}, 0}, // Extended NOP

        // Final DCS commands
        {static_cast<uint8_t>(Command::sleep_out), {}, 120},  // Sleep Out (120ms delay)
        {static_cast<uint8_t>(Command::madctl), {madctl}, 0}, // Memory access control
        {static_cast<uint8_t>(Command::colmod), {0x55}, 0},   // 16-bit/pixel (RGB565)
        {static_cast<uint8_t>(Command::display_on), {}, 20},  // Display ON (20ms delay)
    });

    send_commands(init_commands);

    return true;
  }

  /**
   * @brief Set the display rotation.
   */
  static void rotate(const DisplayRotation &rotation) {
    uint8_t data = 0x00;
    if (swap_color_order_) {
      data |= LCD_CMD_BGR_BIT;
    }
    if (mirror_x_) {
      data |= GS_BIT; // LCD_CMD_MX_BIT;
    }
    if (mirror_y_) {
      data |= SS_BIT; // LCD_CMD_MY_BIT;
    }
    if (swap_xy_) {
      data |= 0; // LCD_CMD_MV_BIT;
    }
    switch (rotation) {
    case DisplayRotation::LANDSCAPE:
      break;
    case DisplayRotation::PORTRAIT:
      if (mirror_portrait_) {
        data ^= GS_BIT; // (LCD_CMD_MX_BIT | LCD_CMD_MV_BIT);
      } else {
        data ^= SS_BIT; // (LCD_CMD_MY_BIT | LCD_CMD_MV_BIT);
      }
      break;
    case DisplayRotation::LANDSCAPE_INVERTED:
      data ^= GS_BIT | SS_BIT; // (LCD_CMD_MY_BIT | LCD_CMD_MX_BIT);
      break;
    case DisplayRotation::PORTRAIT_INVERTED:
      if (mirror_portrait_) {
        data ^= SS_BIT; // (LCD_CMD_MY_BIT | LCD_CMD_MV_BIT);
      } else {
        data ^= GS_BIT; // (LCD_CMD_MX_BIT | LCD_CMD_MV_BIT);
      }
      break;
    }

    auto lcd_commands = std::to_array<display_drivers::DisplayInitCmd<>>({
        // CMD_Page 0 - User Commands
        {static_cast<uint8_t>(Command::page_select),
         {0x98, 0x81, 0x00},
         0},                                                // Switch to Command Page 0
        {static_cast<uint8_t>(Command::madctl), {data}, 0}, // Memory access control
    });
    send_commands(lcd_commands);
  }

  /**
   * @brief Flush LVGL area to display.
   */
  static void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map) {
    fill(disp, area, color_map, (1u << (int)display_drivers::Flags::FLUSH_BIT));
  }

  /**
   * @brief Set drawing area using an lv_area_t.
   */
  static void set_drawing_area(const lv_area_t *area) {
    set_drawing_area(area->x1, area->y1, area->x2, area->y2);
  }

  /**
   * @brief Set drawing area using coordinates.
   */
  static void set_drawing_area(size_t xs, size_t ys, size_t xe, size_t ye) {
    std::array<uint8_t, 4> data;

    int offset_x = 0;
    int offset_y = 0;
    get_offset_rotated(offset_x, offset_y);

    uint16_t start_x = xs + offset_x;
    uint16_t end_x = xe + offset_x;
    uint16_t start_y = ys + offset_y;
    uint16_t end_y = ye + offset_y;

    // column (x)
    data[0] = (start_x >> 8) & 0xFF;
    data[1] = start_x & 0xFF;
    data[2] = (end_x >> 8) & 0xFF;
    data[3] = end_x & 0xFF;
    write_command_(static_cast<uint8_t>(Command::caset), data, 0);

    // row (y)
    data[0] = (start_y >> 8) & 0xFF;
    data[1] = start_y & 0xFF;
    data[2] = (end_y >> 8) & 0xFF;
    data[3] = end_y & 0xFF;
    write_command_(static_cast<uint8_t>(Command::raset), data, 0);
  }

  /**
   * @brief Fill an area with a color map.
   */
  static void fill(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map,
                   uint32_t flags = 0) {
    std::scoped_lock lock{spi_mutex_};
    lv_draw_sw_rgb565_swap(color_map, lv_area_get_width(area) * lv_area_get_height(area));
    if (lcd_send_lines_) {
      int offset_x = 0;
      int offset_y = 0;
      get_offset_rotated(offset_x, offset_y);
      lcd_send_lines_(area->x1 + offset_x, area->y1 + offset_y, area->x2 + offset_x,
                      area->y2 + offset_y, color_map, flags);
    } else {
      set_drawing_area(area);
      uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);
      write_command_(static_cast<uint8_t>(Command::ramwr), {color_map, size * 2}, flags);
    }
  }

  /**
   * @brief Clear a rectangular region to a color.
   */
  static void clear(size_t x, size_t y, size_t width, size_t height, uint16_t color = 0x0000) {
    set_drawing_area(x, y, x + width, y + height);

    uint32_t size = width * height;
    static constexpr int max_words = 1024;
    uint16_t color_words[max_words];
    for (int i = 0; i < max_words; i++)
      color_words[i] = color;
    for (uint32_t i = 0; i < size; i += max_words) {
      uint32_t chunk = std::min<uint32_t>(size - i, max_words);
      write_command_(static_cast<uint8_t>(Command::ramwr),
                     {reinterpret_cast<uint8_t *>(color_words), chunk * 2}, 0);
    }
  }

  /**
   * @brief Send a list of initialization/display commands.
   */
  static void send_commands(std::span<const display_drivers::DisplayInitCmd<>> commands) {
    using namespace std::chrono_literals;
    for (const auto &[cmd, params, delay_ms] : commands) {
      std::scoped_lock lock{spi_mutex_};
      write_command_(cmd, params, 0);
      std::this_thread::sleep_for(delay_ms * 1ms);
    }
  }

  /**
   * @brief Set top-left pixel offset.
   */
  static void set_offset(int x, int y) {
    offset_x_ = x;
    offset_y_ = y;
  }

  /**
   * @brief Get offset.
   */
  static void get_offset(int &x, int &y) {
    x = offset_x_;
    y = offset_y_;
  }

  /**
   * @brief Get offset, adjusted for rotation.
   */
  static void get_offset_rotated(int &x, int &y) {
    auto rotation = lv_display_get_rotation(lv_display_get_default());
    switch (rotation) {
    case LV_DISPLAY_ROTATION_90:
    case LV_DISPLAY_ROTATION_270:
      x = offset_y_;
      y = offset_x_;
      break;
    case LV_DISPLAY_ROTATION_0:
    case LV_DISPLAY_ROTATION_180:
    default:
      x = offset_x_;
      y = offset_y_;
      break;
    }
  }

protected:
  static inline display_drivers::write_command_fn write_command_;
  static inline display_drivers::read_command_fn read_command_;
  static inline display_drivers::send_lines_fn lcd_send_lines_;
  static inline gpio_num_t reset_pin_;
  static inline gpio_num_t dc_pin_;
  static inline int offset_x_;
  static inline int offset_y_;
  static inline bool mirror_x_;
  static inline bool mirror_y_;
  static inline bool mirror_portrait_;
  static inline bool swap_xy_;
  static inline bool swap_color_order_;
  static inline std::mutex spi_mutex_;
};
} // namespace espp
