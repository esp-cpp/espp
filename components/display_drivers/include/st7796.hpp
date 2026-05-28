#pragma once

#include <algorithm>
#include <array>
#include <mutex>
#include <thread>

#include "display_drivers.hpp"

namespace espp {
/**
 * @brief Display driver for the ST7796 display controller.
 *
 * This implementation follows the same lightweight callback-based structure as
 * the other ESPP display drivers, with initialization values adapted from the
 * Smart Panlee SC01 Plus board configuration.
 */
class St7796 {
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

  /// Initialize the display controller state and send the power-on sequence.
  /// \param config Driver transport and orientation configuration.
  static void initialize(const display_drivers::Config &config) {
    write_command_ = config.write_command;
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

    display_drivers::init_pins(reset_pin_, dc_pin_, config.reset_value);

    uint8_t madctl = 0;
    if (swap_color_order_) {
      madctl |= LCD_CMD_BGR_BIT;
    }
    if (mirror_x_) {
      madctl |= LCD_CMD_MX_BIT;
    }
    if (mirror_y_) {
      madctl |= LCD_CMD_MY_BIT;
    }
    if (swap_xy_) {
      madctl |= LCD_CMD_MV_BIT;
    }

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

    if (config.invert_colors) {
      write_command_(static_cast<uint8_t>(Command::invon), {}, 0);
    } else {
      write_command_(static_cast<uint8_t>(Command::invoff), {}, 0);
    }
  }

  /// Update the controller addressing mode for a new display rotation.
  /// \param rotation New display rotation.
  static void rotate(const DisplayRotation &rotation) {
    uint8_t data = 0;
    if (swap_color_order_) {
      data |= LCD_CMD_BGR_BIT;
    }
    if (mirror_x_) {
      data |= LCD_CMD_MX_BIT;
    }
    if (mirror_y_) {
      data |= LCD_CMD_MY_BIT;
    }
    if (swap_xy_) {
      data |= LCD_CMD_MV_BIT;
    }
    switch (rotation) {
    case DisplayRotation::LANDSCAPE:
      break;
    case DisplayRotation::PORTRAIT:
      if (mirror_portrait_) {
        data ^= (LCD_CMD_MX_BIT | LCD_CMD_MV_BIT);
      } else {
        data ^= (LCD_CMD_MY_BIT | LCD_CMD_MV_BIT);
      }
      break;
    case DisplayRotation::LANDSCAPE_INVERTED:
      data ^= (LCD_CMD_MY_BIT | LCD_CMD_MX_BIT);
      break;
    case DisplayRotation::PORTRAIT_INVERTED:
      if (mirror_portrait_) {
        data ^= (LCD_CMD_MY_BIT | LCD_CMD_MV_BIT);
      } else {
        data ^= (LCD_CMD_MX_BIT | LCD_CMD_MV_BIT);
      }
      break;
    }
    std::scoped_lock lock{spi_mutex_};
    write_command_(static_cast<uint8_t>(Command::madctl), {&data, 1}, 0);
  }

  /// Flush an LVGL drawing area to the display.
  /// \param disp LVGL display pointer.
  /// \param area Area to flush.
  /// \param color_map RGB565 pixel data for the area.
  static void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *color_map) {
    fill(disp, area, color_map, (1 << (int)display_drivers::Flags::FLUSH_BIT));
  }

  /// Set the active drawing area using an LVGL area.
  /// \param area Area to make active on the controller.
  static void set_drawing_area(const lv_area_t *area) {
    set_drawing_area(area->x1, area->y1, area->x2, area->y2);
  }

  /// Set the active drawing area using raw coordinates.
  /// \param xs Starting x coordinate.
  /// \param ys Starting y coordinate.
  /// \param xe Ending x coordinate.
  /// \param ye Ending y coordinate.
  static void set_drawing_area(size_t xs, size_t ys, size_t xe, size_t ye) {
    std::array<uint8_t, 4> data;

    int offset_x = 0;
    int offset_y = 0;
    get_offset_rotated(offset_x, offset_y);

    uint16_t start_x = xs + offset_x;
    uint16_t end_x = xe + offset_x;
    uint16_t start_y = ys + offset_y;
    uint16_t end_y = ye + offset_y;

    data[0] = (start_x >> 8) & 0xFF;
    data[1] = start_x & 0xFF;
    data[2] = (end_x >> 8) & 0xFF;
    data[3] = end_x & 0xFF;
    write_command_(static_cast<uint8_t>(Command::caset), data, 0);

    data[0] = (start_y >> 8) & 0xFF;
    data[1] = start_y & 0xFF;
    data[2] = (end_y >> 8) & 0xFF;
    data[3] = end_y & 0xFF;
    write_command_(static_cast<uint8_t>(Command::raset), data, 0);
  }

  /// Fill an LVGL area using the supplied color buffer.
  /// \param disp LVGL display pointer.
  /// \param area Area to fill.
  /// \param color_map RGB565 pixel data for the area.
  /// \param flags Optional transport flags.
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

  /// Clear a rectangular region to a solid color.
  /// \param x Starting x coordinate.
  /// \param y Starting y coordinate.
  /// \param width Width in pixels.
  /// \param height Height in pixels.
  /// \param color RGB565 fill color.
  static void clear(size_t x, size_t y, size_t width, size_t height, uint16_t color = 0x0000) {
    if (width == 0 || height == 0) {
      return;
    }

    set_drawing_area(x, y, x + width - 1, y + height - 1);

    uint32_t size = width * height;
    static constexpr int max_pixels_to_send = 1024;
    std::array<uint16_t, max_pixels_to_send> color_data;
    std::fill(color_data.begin(), color_data.end(), color);
    for (int i = 0; i < size; i += max_pixels_to_send) {
      size_t num_pixels = std::min((int)(size - i), max_pixels_to_send);
      write_command_(static_cast<uint8_t>(Command::ramwr),
                     {reinterpret_cast<uint8_t *>(color_data.data()), num_pixels * 2}, 0);
    }
  }

  /// Send a sequence of initialization commands to the controller.
  /// \param commands Command list to send in order.
  static void send_commands(std::span<const display_drivers::DisplayInitCmd<>> commands) {
    using namespace std::chrono_literals;

    for (const auto &[command, parameters, delay_ms] : commands) {
      std::scoped_lock lock{spi_mutex_};
      write_command_(command, parameters, 0);
      std::this_thread::sleep_for(delay_ms * 1ms);
    }
  }

  /// Set the controller coordinate offset used by this driver.
  /// \param x X offset in pixels.
  /// \param y Y offset in pixels.
  static void set_offset(int x, int y) {
    offset_x_ = x;
    offset_y_ = y;
  }

  /// Get the configured controller coordinate offset.
  /// \param x Filled with the x offset.
  /// \param y Filled with the y offset.
  static void get_offset(int &x, int &y) {
    x = offset_x_;
    y = offset_y_;
  }

  /// Get the coordinate offset after accounting for the current LVGL rotation.
  /// \param x Filled with the rotated x offset.
  /// \param y Filled with the rotated y offset.
  static void get_offset_rotated(int &x, int &y) {
    auto *display = lv_display_get_default();
    auto rotation = display ? lv_display_get_rotation(display) : LV_DISPLAY_ROTATION_0;
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
